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

#include <map>
#include <string>
#include <vector>

#include "../String/String.h"

class CommandLineParser {

public:
    CommandLineParser(int argc, char** argv) {
        for (int currentIndex = 1; currentIndex < argc; ++currentIndex) {
            if (argv[currentIndex][0] == '-') {
                std::string key(&argv[currentIndex][1]);
                std::string value("");
                if ((currentIndex + 1) < argc) value.assign(argv[currentIndex + 1]);
                arguments[key] = value;
            }
        }
    }

    CommandLineParser(const std::string& input) {
        std::vector<std::string> argv = String::split(String::trim(input), ' ');
        for (size_t currentIndex = 1; currentIndex < argv.size(); ++currentIndex) {
            if (argv[currentIndex][0] == '-') {
                std::string key(&argv[currentIndex][1]);
                std::string value("");
                if ((currentIndex + 1) < argv.size()) value.assign(argv[currentIndex + 1]);
                arguments[key] = value;
            }
        }
    }

    template<typename T>
    inline T value(const std::string key, const T defaultValue = T()) noexcept {
        if (arguments.find(key) != arguments.end()) {
            return String::lexicalCast<T>(arguments[key]);
        } else {
            return defaultValue;
        }
    }

    template<typename T>
    inline T get(const std::string key, const T defaultValue = T()) noexcept {
        return value<T>(key, defaultValue);
    }

    inline bool isSet(const std::string key) const noexcept {
        return arguments.find(key) != arguments.end();
    }

    inline size_t numberOfArguments() const noexcept {
        return arguments.size();
    }

private:
    std::map<std::string, std::string> arguments;

};
