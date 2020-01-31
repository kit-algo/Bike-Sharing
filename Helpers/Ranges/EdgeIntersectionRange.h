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

#include "../Assert.h"
#include "../Meta.h"
#include "../Types.h"

#include "../../DataStructures/Attributes/Attributes.h"

template<typename GRAPH_A, typename GRAPH_B>
class EdgeIntersectionRange {

public:
    using GraphA = GRAPH_A;
    using GraphB = GRAPH_B;
    using Type = EdgeIntersectionRange<GraphA, GraphB>;

private:
    using RangeA = decltype(std::declval<const GraphA*>()->edgesFrom(std::declval<Vertex>()));
    using RangeB = decltype(std::declval<const GraphB*>()->edgesFrom(std::declval<Vertex>()));

    using IteratorA = decltype(std::declval<RangeA>().begin());
    using IteratorB = decltype(std::declval<RangeB>().begin());

public:
    class Iterator {
    public:
        Iterator(const EdgeIntersectionRange* const outer, const IteratorA& edgeA, const IteratorB& edgeB) : outer(outer), edgeA(edgeA), edgeB(edgeB) {}
        inline bool operator!=(const Iterator& other) const noexcept {return edgeA != other.edgeA;}
        inline Edge operator*() const noexcept {return *edgeA;}
        inline Iterator& operator++() noexcept {++edgeA; ++edgeB; validate(); return *this;}
        inline Iterator& operator+=(const size_t n) noexcept {for (size_t j = 0; j < n; j++) ++(*this); return *this;}
        inline Iterator operator+(const size_t n) const noexcept {return Iterator(*this) += n;}
        inline Edge operator[](const size_t n) const noexcept {return *(*this + n);}
        inline Iterator& validate() noexcept {
            if (!(edgeB != outer->rangeB.end())) {
                edgeA = outer->rangeA.end();
                return *this;
            }
            while ((edgeA != outer->rangeA.end()) && (outer->graphA->get(ToVertex, *edgeA) != outer->graphB->get(ToVertex, *edgeB))) {
                if (outer->graphA->get(ToVertex, *edgeA) < outer->graphB->get(ToVertex, *edgeB)) {
                    ++edgeA;
                } else {
                    ++edgeB;
                    if (!(edgeB != outer->rangeB.end())) {
                        edgeA = outer->rangeA.end();
                        return *this;
                    }
                }
            }
            return *this;
        }
    private:
        const EdgeIntersectionRange* outer;
        IteratorA edgeA;
        IteratorB edgeB;
    };

    EdgeIntersectionRange() :
        graphA(nullptr),
        graphB(nullptr) {
    }

    EdgeIntersectionRange(const GraphA* const graphA, const GraphB* const graphB, const Vertex vertex) :
        graphA(graphA),
        graphB(graphB),
        rangeA(graphA->edgesFrom(vertex)),
        rangeB(graphB->edgesFrom(vertex)) {
    }

    inline Iterator begin() const noexcept {
        return Iterator(this, rangeA.begin(), rangeB.begin()).validate();
    }

    inline Iterator end() const noexcept {
        return Iterator(this, rangeA.end(), rangeB.end());
    }

    inline bool empty() const noexcept {
        return !(begin() != end());
    }

    inline size_t size() const noexcept {
        size_t result = 0;
        for (Iterator i = begin(); i != end(); ++i) {
            result++;
        }
        return result;
    }

    inline Edge operator[](const size_t i) const noexcept {
        AssertMsg(i < size(), "Index " << i << " is out of range!");
        return begin()[i];
    }

    inline Edge front() const noexcept {
        AssertMsg(!empty(), "Range is empty!");
        return *(begin());
    }

    inline Edge back() const noexcept {
        AssertMsg(!empty(), "Range is empty!");
        Edge result;
        for (Iterator i = begin(); i != end(); ++i) {
            result = *i;
        }
        return result;
    }

private:
    const GraphA* graphA;
    const GraphB* graphB;

    RangeA rangeA;
    RangeB rangeB;

};
