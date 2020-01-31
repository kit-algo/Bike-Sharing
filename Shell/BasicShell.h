/**********************************************************************************

 Copyright (c) 2020 Tobias Zündorf

 MIT License

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
 files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
 modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 is furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

**********************************************************************************/

#pragma once

#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <sys/ioctl.h>

#include "Command.h"

#include "LineBuffer.h"

#include "../Helpers/Timer.h"
#include "../Helpers/HighlightText.h"
#include "../Helpers/FileSystem/FileSystem.h"
#include "../Helpers/String/String.h"
#include "../Helpers/Vector/Vector.h"

namespace Shell {

const std::string newLine = "\n\r";

inline char getch() {
    char buf = 0;
    termios old;
    memset(&old, 0, sizeof(old));
    if (tcgetattr(0, &old) < 0) perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0) perror ("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) perror ("tcsetattr ~ICANON");
    return (buf);
}

class BasicShell {

public:
    BasicShell(std::string programName = "", std::string prompt = "> ", bool autosaveCach = true, std::string cachFile = "readcach", std::ostream& out = std::cout) :
        programName(programName),
        prompt(prompt),
        out(out),
        shell(*this),
        running(false),
        cachPos(-1),
        cachFile(cachFile),
        autosaveCach(autosaveCach),
        lineBuffer(out),
        autoCompleteFiles(true),
        reportCommandTimes(false),
        reportParameters(false) {
        if (programName != "") this->cachFile = programName + "." + cachFile;
        dir = FileSystem::getWorkingDirectory();
        if (autosaveCach) loadCach();
    }

    ~BasicShell() {
        if (autosaveCach) saveCach();
        while (!commands.empty()) {
            delete commands.back();
            commands.pop_back();
        }
    }

    inline std::string readLine() {
        printPrompt();
        lineBuffer.setFirstCharColumn(prompt.size());
        while (true) {
            char c = readChar();
            if (c == '\n') return acceptLineBuffer();
            lineBuffer << c;
        }
        return "";
    }

    inline char readChar() {
        while (true) {
            uint64_t c = getch();
            if ((c >= 32 && c <= 126) || (c >= 128 && c <= 254)) { // Displayable character
                return (char)c;
            } else if (c == '\n') { // New line
                return '\n';
            } else if (c == 27) { // Escape sequence
                uint64_t c1 = getch();
                uint64_t c2 = getch();
                c = (c1 * 128) + c2;
                if (c2 >= 50 && c2 <= 55) {
                    uint64_t c3 = getch();
                    c = (c * 128) + c3;
                }
                if (c == 10184) { // Pos1 Key
                    lineBuffer.begin();
                } else if (c == 10182) { // End Key
                    lineBuffer.end();
                } else if (c == 11713) { // Up Key
                    cachUp();
                } else if (c == 11714) { // Down Key
                    cachDown();
                } else if (c == 11715) { // Right Key
                    if (!lineBuffer.right()) {
                        autoCompleteCach(lineBuffer.getPrefix());
                    }
                } else if (c == 11716) { // Left Key
                    lineBuffer.left();
                } else if (c == 1497598) { // Delete
                    lineBuffer.deleteChar();
                } else { // Other
                    shell << "\r\x1b[2K";
                    yellow("Unknown key '" + std::to_string((int)(c)) + "'");
                    shell << "\n";
                    printPrompt();
                    lineBuffer.rePrint();
                }
            } else if (c == 9) { // Tab
                autoComplete();
            } else if (c == 127) { // Backspace
                lineBuffer.backspace();
            } else if (c == 24) { // ctrl + x
                saveCach();
            } else { // Other
                shell << "\r\x1b[2K";
                yellow("Unknown key '" + std::to_string((int)(c)) + "'");
                shell << "\n";
                printPrompt();
                lineBuffer.rePrint();
            }
        }
        return '\n';
    }

    inline std::string acceptLineBuffer() {
        std::string s = lineBuffer.accept();
        addToCach(s);
        return s;
    }

    inline void autoComplete() {
        std::vector<std::string> suggestions;
        std::string currentLine = lineBuffer.getPrefix();
        currentLine = String::trim(currentLine);
        const size_t firstSpacePos = currentLine.find(' ');
        const size_t lastSpacePos = currentLine.rfind(' ');
        if (firstSpacePos < currentLine.size()) {
            const std::string commandName = currentLine.substr(0, firstSpacePos);
            currentLine = currentLine.substr(lastSpacePos + 1);
            collectParameterSuggestions(commandName, currentLine, suggestions, String::count(String::trim(currentLine), ' ') - 1);
        } else {
            collectCommandSuggestions(currentLine, suggestions);
        }
        collectPathSuggestions(currentLine, suggestions);
        collectAdditionalSuggestions(currentLine, suggestions);
        autoComplete(currentLine, suggestions);
    }

    inline void cachUp() {
        cachPos--;
        if (cachPos < 0) cachPos = cach.size() - 1;
        if (cachPos >= 0) lineBuffer.setString(cach[cachPos]);
    }

    inline void cachDown() {
        if (cachPos < 0) return;
        cachPos++;
        if (size_t(cachPos) >= cach.size()) {
            cachPos = -1;
            lineBuffer.setString("");
        } else {
            lineBuffer.setString(cach[cachPos]);
        }
    }

    inline void addToCach(const std::string& s) {
        if (s.size() < 1) return;
        std::string line = String::trim(s);
        if (cach.empty() || cach.back() != line) cach.push_back(line);
        cachPos = -1;
    }

    inline Command* findCommand(const std::string& name) {
        for (Command* command : commands) {
            if (command->matches(name)) {
                return command;
            }
        }
        return nullptr;
    }

    inline const Command* findCommand(const std::string& name) const {
        for (const Command* command : commands) {
            if (command->matches(name)) {
                return command;
            }
        }
        return nullptr;
    }

    void run() {
        running = true;
        while (running) {
            interpretCommand(readLine());
        }
    }

    inline void interpretCommand(const std::string& line) noexcept {
        std::vector<std::string> tokens = splitCommand(line);
        if (tokens[0].size() < 1) return;
        Command* command = findCommand(tokens[0]);
        if (command != nullptr) {
            if (autosaveCach) saveCach();
            Timer commandTimer;
            command->execute(tokens[1]);
            if (reportCommandTimes) shell << grey("[Finished in ", String::msToString(commandTimer.elapsedMilliseconds()), "]") << newLine;
        } else {
            shell << "Unknown command: \"" << tokens[0] << "\", try using \"help\"" << newLine;
        }
    }

    template<typename T>
    inline T ask(const std::string& question) {
        std::string oldPrompt = prompt;
        prompt = question;
        std::string line = readLine();
        prompt = oldPrompt;
        return String::lexicalCast<T>(line);
    }

    inline std::string ask(const std::string& question) {
        return ask<std::string>(question);
    }

    template<typename T>
    inline T ask(const std::string& question, const std::vector<std::string>& suggestions) {
        additionalSuggestions = suggestions;
        std::string oldPrompt = prompt;
        prompt = question;
        std::string line = readLine();
        prompt = oldPrompt;
        additionalSuggestions.clear();
        return String::lexicalCast<T>(line);
    }

    inline std::string ask(const std::string& question, const std::vector<std::string>& suggestions) {
        return ask<std::string>(question, suggestions);
    }

    inline void stop() {
        running = false;
    }

    inline void execute(const std::string& line) noexcept {
        printPrompt() << line << newLine;
        addToCach(line);
        interpretCommand(line);
    }

    inline void loadCach() {
        std::ifstream inputFile(getCachFile());
        if (!inputFile.is_open()) return;
        while (!inputFile.eof()) {
            std::string line;
            getline(inputFile, line);
            addToCach(line);
        }
        inputFile.close();
    }

    inline void saveCach() const {
        std::ofstream outputFile(getCachFile());
        for (const std::string& s : cach) {
            outputFile << s << std::endl;
        }
        outputFile.close();
    }

    template<typename T>
    inline BasicShell& operator<<(const T& c) {
        out << c << std::flush;
        return *this;
    }

    inline BasicShell& flush() {
        out << std::flush;
        return *this;
    }

    inline BasicShell& endl() {
        out << std::endl;
        return *this;
    }

    template<typename... T>
    inline BasicShell& error(const T&... c)    {return shell << (std::string)::error(c...);}
    template<typename... T>
    inline BasicShell& red(const T&... c)      {return shell << (std::string)::red(c...);}
    template<typename... T>
    inline BasicShell& green(const T&... c)    {return shell << (std::string)::green(c...);}
    template<typename... T>
    inline BasicShell& yellow(const T&... c)   {return shell << (std::string)::yellow(c...);}
    template<typename... T>
    inline BasicShell& blue(const T&... c)     {return shell << (std::string)::blue(c...);}
    template<typename... T>
    inline BasicShell& magenta(const T&... c)  {return shell << (std::string)::magenta(c...);}
    template<typename... T>
    inline BasicShell& cyan(const T&... c)     {return shell << (std::string)::cyan(c...);}
    template<typename... T>
    inline BasicShell& white(const T&... c)    {return shell << (std::string)::white(c...);}


    inline BasicShell& printPrompt() {return blue(prompt);}

public:
    inline bool addCommand(Command* c) {
        if (Vector::contains(commands, c)) {
            return false;
        } else {
            commands.push_back(c);
            return true;
        }
    }

    inline const std::vector<Command*>& getCommands() const {
        return commands;
    }

    inline std::string getProgramName() const {
        return programName;
    }

    inline void setProgramName(const std::string& name) {
        programName = name;
    }

    inline std::string getPrompt() const {
        return prompt;
    }

    inline void setPrompt(const std::string& p) {
        prompt = p;
    }

    inline void clearLine() const {
        shell << "\r" << whiteSpace(getScreenWidth()) << "\r";
    }

    inline std::string getDir() const {
        return dir;
    }

    inline void setDir(const std::string& s) {
        dir = s;
        while(dir.rfind('/') == dir.size() - 1) {
            dir = dir.substr(0, dir.size() - 1);
        }
    }

    inline std::vector<std::string>& getReadCach() {
        return cach;
    }

    inline std::string getCachFile() const {
        return cachFile;
    }

    inline void setCachFile(const std::string& filename) {
        cachFile = filename;
    }

    inline bool getAutosaveCach() const {
        return autosaveCach;
    }

    inline void setAutosaveCach(const bool b) {
        autosaveCach = b;
    }

    inline bool getAutoCompleteFiles() const {
        return autoCompleteFiles;
    }

    inline void setAutoCompleteFiles(const bool b) {
        autoCompleteFiles = b;
    }

    inline bool getReportCommandTimes() const {
        return reportCommandTimes;
    }

    inline void setReportCommandTimes(const bool b) {
        reportCommandTimes = b;
    }

    inline bool getReportParameters() const {
        return reportParameters;
    }

    inline void setReportParameters(const bool b) {
        reportParameters = b;
    }

private:
    inline std::string whiteSpace(int size) const {
        std::stringstream ss;
        for (int i = 0; i < size; i++) {
            ss << " ";
        }
        return ss.str();
    }

    inline std::vector<std::string> splitCommand(const std::string& s) const {
        int splitPos = s.find(' ');
        std::vector<std::string> result;
        if (splitPos < 0) {
            result.push_back(s);
            result.push_back("");
        } else {
            result.push_back(s.substr(0, splitPos));
            result.push_back(s.substr(splitPos + 1));
        }
        return result;
    }

    inline void collectCommandSuggestions(const std::string& s, std::vector<std::string>& suggestions) {
        for (Command* command : commands) {
            if (command->name().find(s) == 0) {
                suggestions.push_back(command->name());
            }
        }
    }

    inline void collectAdditionalSuggestions(const std::string& s, std::vector<std::string>& suggestions) {
        for (const std::string& suggestion : additionalSuggestions) {
            if (suggestion.find(s) == 0) {
                suggestions.push_back(suggestion);
            }
        }
    }

    inline void collectPathSuggestions(const std::string& s, std::vector<std::string>& suggestions) {
        int splitPos = s.rfind('/');
        std::string path;
        std::string file;
        if (splitPos < 0) {
            path = getDir();
            file = s;
        } else {
            path = FileSystem::extendPath(getDir(), s.substr(0, splitPos));
            file = s.substr(splitPos + 1);
        }
        std::vector<std::string> files = FileSystem::getFiles(path);
        for (const std::string& fileName : files) {
            if (fileName.empty()) continue;
            if (fileName.find(file) == 0) {
                std::string suggestion = path + "/" + fileName;
                if (fileName[0] != '.' && FileSystem::isDirectory(suggestion)) suggestion += "/";
                suggestions.push_back(suggestion);
            }
        }
    }

    inline void collectCachSuggestions(const std::string& s, std::vector<std::string>& suggestions) {
        std::string line = String::trim(s);
        if (s.size() > 0 && String::isWhiteSpace(s.back())) line += ' ';
        for (const std::string& entry : cach) {
            if (entry.find(line) == 0 && !Vector::contains(suggestions, entry)) {
                suggestions.push_back(entry);
            }
        }
    }

    inline void collectParameterSuggestions(const std::string& commandName, const std::string& s, std::vector<std::string>& suggestions) {
        Command* command = findCommand(commandName);
        if (command != nullptr) {
            for (const std::string& entry : command->parameterSuggestions()) {
                if (entry.find(s) == 0) {
                    suggestions.push_back(entry);
                }
            }
        }
    }

    inline void collectParameterSuggestions(const std::string& commandName, const std::string& s, std::vector<std::string>& suggestions, const size_t index) {
        Command* command = findCommand(commandName);
        if (command != nullptr) {
            for (const std::string& entry : command->parameterSuggestions(index)) {
                if (entry.find(s) == 0) {
                    suggestions.push_back(entry);
                }
            }
        }
    }

    inline void autoCompleteCommand(const std::string& s) {
        std::vector<std::string> suggestions;
        collectCommandSuggestions(s, suggestions);
        autoComplete(s, suggestions);
    }

    inline void autoCompletePath(const std::string& s) {
        std::vector<std::string> suggestions;
        collectPathSuggestions(s, suggestions);
        autoComplete(s, suggestions);
    }

    inline void autoCompleteCommandOrPath(const std::string& s) {
        std::vector<std::string> suggestions;
        collectCommandSuggestions(s, suggestions);
        collectPathSuggestions(s, suggestions);
        autoComplete(s, suggestions);
    }

    inline void autoCompleteCach(const std::string& s) {
        std::vector<std::string> suggestions;
        collectCachSuggestions(s, suggestions);
        autoComplete(s, suggestions);
    }

    inline void autoComplete(const std::string& s, std::vector<std::string>& suggestions) {
        if (suggestions.size() == 0) return;
        if (suggestions.size() == 1) {
            autoComplete(s, suggestions[0]);
            return;
        }
        std::sort(suggestions.begin(), suggestions.end());
        size_t minSize = suggestions[0].size();
        std::vector<size_t> offsets(suggestions.size(), 0);
        for (size_t i = 0; i < suggestions.size(); i++) {
            offsets[i] = suggestions[i].rfind(s);
            if (offsets[i] > suggestions[i].size()) {
                offsets[i] = suggestions[i].size() - s.size();
            }
            if (minSize > suggestions[i].size() - offsets[i]) {
                minSize = suggestions[i].size() - offsets[i];
            }
        }
        size_t i = s.size();
        bool equal = (i < minSize);
        while (i < minSize && equal) {
            char c = suggestions[0][offsets[0] + i];
            for (size_t j = 1; j < suggestions.size(); j++) {
                if (c != suggestions[j][offsets[j] + i]) {
                    equal = false;
                    break;
                }
            }
            if (equal) i++;
        }
        if ((!equal) && (i <= s.size())) {
            shell << lineBuffer.getSuffix() << newLine;
            for (const std::string& n : suggestions) {
                shell << n << newLine;
            }
            printPrompt();
            lineBuffer.rePrint();
        } else {
            autoComplete(s, suggestions[0].substr(offsets[0], i));
        }
    }

    inline void autoComplete(const std::string& s, const std::string& suggestion) {
        if (s.empty()) return;
        std::string oldLine = lineBuffer.getPrefix();
        size_t index = oldLine.rfind(s);
        if (index >= oldLine.size()) return;
        std::string newLine = oldLine.substr(0, index) + suggestion;
        if (index + s.size() < oldLine.size()) newLine = newLine + oldLine.substr(index + s.size());
        lineBuffer.setPrefix(newLine);
    }

private:
    std::string programName;
    std::string prompt;
    std::ostream& out;
    BasicShell& shell;
    bool running;

    std::vector<Command*> commands;

    std::vector<std::string> cach;
    int cachPos;
    std::string cachFile;
    bool autosaveCach;

    LineBuffer lineBuffer;

    std::string dir;
    bool autoCompleteFiles;

    bool reportCommandTimes;
    bool reportParameters;

    std::vector<std::string> additionalSuggestions;

};

}
