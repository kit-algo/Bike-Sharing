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

#include <iostream>
#include <string>
#include <sstream>

class StreamWrapper {

public:
    StreamWrapper() :
        flushed(false) {
    }

    StreamWrapper(const StreamWrapper& stream) :
        flushed(false) {
        stream.flush(text);
    }

    ~StreamWrapper() {
        flush(std::cout);
        std::cout << std::flush;
    }

    template<typename T>
    inline void append(const T& t) const {
        text.precision(std::cout.precision());
        text << t;
    }

    template<typename T>
    inline std::ostream& operator<<(const T& t) const {
        flush(std::cout);
        return std::cout << t << std::flush;
    }

    inline std::ostream& operator<<(std::ostream& (*manip)(std::ostream&)) {
        flush(std::cout);
        manip(std::cout);
        return std::cout;
    }

    inline operator std::string() const {
        std::stringstream ss;
        flush(ss);
        return ss.str();
    }

protected:
    inline void flush(std::ostream& stream) const {
        if (!flushed) {
            stream << text.str();
            flushed = true;
        }
    }

private:
    mutable std::stringstream text;
    mutable bool flushed;

};

inline std::ostream& operator<<(std::ostream& out, const StreamWrapper& stream) {
    return out << (std::string)stream;
}

inline void concatenate(StreamWrapper&) {}

template<typename T, typename... U>
inline void concatenate(StreamWrapper& stream, const T& t, const U&... u) {
    stream.append(t);
    concatenate(stream, u...);
}

template<typename... T>
inline StreamWrapper concatenate(const T&... t) {
    StreamWrapper stream;
    concatenate(stream, t...);
    return stream;
}

template<typename... T>
inline StreamWrapper red(const T&... t)     {return concatenate("\033[1;31m", t..., "\033[0m");}
template<typename... T>
inline StreamWrapper green(const T&... t)   {return concatenate("\033[1;32m", t..., "\033[0m");}
template<typename... T>
inline StreamWrapper yellow(const T&... t)  {return concatenate("\033[1;33m", t..., "\033[0m");}
template<typename... T>
inline StreamWrapper blue(const T&... t)    {return concatenate("\033[1;34m", t..., "\033[0m");}
template<typename... T>
inline StreamWrapper magenta(const T&... t) {return concatenate("\033[1;35m", t..., "\033[0m");}
template<typename... T>
inline StreamWrapper cyan(const T&... t)    {return concatenate("\033[1;36m", t..., "\033[0m");}
template<typename... T>
inline StreamWrapper white(const T&... t)   {return concatenate("\033[1;37m", t..., "\033[0m");}
template<typename... T>
inline StreamWrapper grey(const T&... t)   {return concatenate("\033[1;30m", t..., "\033[0m");}

template<typename... T>
inline StreamWrapper warning(const T&... t) {return concatenate("\n\033[33mWARNING: ", t..., "\033[0m\n");}
template<typename... T>
inline StreamWrapper error(const T&... t)   {return concatenate("\n\033[31mERROR: ", t..., "\033[0m\n");}