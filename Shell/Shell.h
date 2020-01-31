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

#include <vector>
#include <string>
#include <sstream>
#include <algorithm>

#include "BasicShell.h"
#include "ParameterizedCommand.h"

#include "../Helpers/Assert.h"
#include "../Helpers/FileSystem/FileSystem.h"

namespace Shell {

class Quit : public Command {

public:
    Quit(BasicShell& shell) : shell(shell) {
        shell.addCommand(this);
    }

    virtual bool matches(const std::string& s) const noexcept {
        std::string in = String::toLower(s);
        return (in == "q") || (in == "quit") || (in == "exit");
    }

    virtual std::string name() const noexcept {
        return "quit";
    }

    virtual std::string helpText() const noexcept {
        return "Type ['Quit' | 'quit' | 'Q' | 'q' | 'Exit' | 'exit'] to terminate the application.";
    }

    virtual void execute(const std::string&) noexcept {
        shell << newLine;
        shell.setReportCommandTimes(false);
        shell.stop();
        shell.getReadCach().pop_back();
    }

private:
    BasicShell& shell;

};

class Help : public Command {

public:
    Help(BasicShell& shell) : shell(shell) {
        shell.addCommand(this);
    }

    virtual bool matches(const std::string& s) const noexcept {
        std::string in = String::toLower(s);
        return (in == "h") || (in == "help");
    }

    virtual std::string name() const noexcept {
        return "help";
    }

    virtual std::string helpText() const noexcept {
        return "Type ['Help' | 'help' | 'H' | 'h'] to get an overview over available commands.\nType ['Help' | 'help' | 'H' | 'h'] <Command> to get detailed help for a command.";
    }

    virtual void execute(const std::string& parameter) noexcept {
        if (parameter == "") {
            const std::vector<Command*> commands = shell.getCommands();
            shell << "Available Commands:" << newLine;
            for (const Command* com : commands) {
                shell << "    " << com->name() << newLine;
            }
        } else {
            std::vector<std::string> tokens = String::split(parameter, ' ');
            const Command* com = shell.findCommand(tokens[0]);
            if (com != nullptr) {
                shell << com->name() << ":" << newLine;
                shell << "    " << String::replaceAll(com->helpText(), '\n', "\n    ") << newLine;
            } else {
                std::cout << "Unknown command: \"" << tokens[0] << "\"." << newLine;
            }
        }
    }

    virtual std::vector<std::string> parameterSuggestions() const {
        std::vector<std::string> suggestions;
        for (Command* command : shell.getCommands()) {
            suggestions.push_back(command->name());
        }
        return suggestions;
    }

private:
    BasicShell& shell;

};

class Dir : public ParameterizedCommand {

public:
    Dir(BasicShell& shell) :
        ParameterizedCommand(shell, "dir", "Displays the current working directory.") {
        shell.addCommand(this);
    }

    virtual void execute() noexcept {
        shell << shell.getDir() << newLine;
    }

};

class Ls : public ParameterizedCommand {

public:
    Ls(BasicShell& shell) :
        ParameterizedCommand(shell, "ls", "Displays all files in the current working directory.") {
    }

    virtual void execute() noexcept {
        std::string path = shell.getDir();
        shell << shell.getDir() << newLine;
        DIR* dir;
        struct dirent* ent;
        if ((dir = opendir(path.c_str())) != nullptr) {
            std::vector<std::string> dirs;
            while ((ent = readdir(dir)) != nullptr) {
                dirs.push_back(ent->d_name);
            }
            closedir(dir);
            std::sort(dirs.begin(), dirs.end());
            for (const std::string& name : dirs) {
                shell << name << newLine;
            }
        } else {
            shell << "Could not open directory: \"" << path << "\"." << newLine;
        }
    }

};

class Cd : public ParameterizedCommand {

public:
    Cd(BasicShell& shell) :
        ParameterizedCommand(shell, "cd", "Changes the current working directory of the shell.") {
        addParameter("Directory");
    }

    virtual void execute() noexcept {
        std::string path = FileSystem::extendPath(shell.getDir(), getParameter("Directory"));
        if (FileSystem::isDirectory(path)) {
            shell.setDir(path);
            shell << shell.getDir() << newLine;
        } else {
            error(path);
        }
    }

private:
    inline void error(const std::string& s) {
        shell << "Unknown path / wrong syntax (" << s << ")." << newLine;
    }

};

class RunScript : public ParameterizedCommand {

public:
    RunScript(BasicShell& shell) :
        ParameterizedCommand(shell, "runScript", "Runs all the commands in the script file.") {
        addParameter("Script file");
    }

    virtual void execute() noexcept {
        const std::string filename = FileSystem::extendPath(shell.getDir(), getParameter("Script file"));
        std::ifstream script(filename);
        AssertMsg(script.is_open(), "cannot open file: " << filename);
        while (!script.eof()) {
            std::string line;
            getline(script, line);
            if (line == "") continue;
            shell.printPrompt();
            shell << line << newLine;
            shell.interpretCommand(line);
        }
    }

};

class ToggleCommandTimeReporting : public ParameterizedCommand {

public:
    ToggleCommandTimeReporting(BasicShell& shell) :
        ParameterizedCommand(shell, "toggleCommandTimeReporting", "Toggles whether the execution time of commands is reported or not.") {
        size_t toggleCount = 0;
        for (const std::string& s : shell.getReadCach()) {
            if (s == name()) {
                toggleCount++;
            }
        }
        if ((toggleCount % 2) != 0) {
            shell.setReportCommandTimes(!shell.getReportCommandTimes());
        }
    }

    virtual void execute() noexcept {
        if (shell.getReportCommandTimes()) {
            shell.setReportCommandTimes(false);
            shell << "Command execution times will no longer be reported!" << newLine;
        } else {
            shell.setReportCommandTimes(true);
            shell << "Command execution times will now be reported!" << newLine;
        }
    }

};

class ToggleParameterReporting : public ParameterizedCommand {

public:
    ToggleParameterReporting(BasicShell& shell) :
        ParameterizedCommand(shell, "toggleParameterReporting", "Toggles whether commands print their parameter values ore not.") {
        size_t toggleCount = 0;
        for (const std::string& s : shell.getReadCach()) {
            if (s == name()) {
                toggleCount++;
            }
        }
        if ((toggleCount % 2) != 0) {
            shell.setReportParameters(!shell.getReportParameters());
        }
    }

    virtual void execute() noexcept {
        if (shell.getReportParameters()) {
            shell.setReportParameters(false);
            shell << "Parameters and their values will no longer be reported!" << newLine;
        } else {
            shell.setReportParameters(true);
            shell << "Parameters and their values will now be reported!" << newLine;
        }
    }

};

class Shell : public BasicShell {

public:
    Shell(std::string programName = "", std::string prompt = "> ") : BasicShell(programName, prompt) {
        new Quit(*this);
        new Help(*this);
        new Dir(*this);
        new Ls(*this);
        new Cd(*this);
        new RunScript(*this);
        new ToggleCommandTimeReporting(*this);
        new ToggleParameterReporting(*this);
    }

};

}
