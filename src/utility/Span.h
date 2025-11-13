#pragma once

#include <cstddef>

namespace gsl {
  template <typename T>
  class span {
  public:
    using element_type = T;
    using pointer = T*;
    using reference = T&;
    using iterator = T*;
    using const_iterator = const T*;

    span() : ptr {nullptr}, len {0} {}
    span(T* data, std::size_t size) : ptr {data}, len {size} {}

    template <std::size_t N>
    span(T (&arr)[N]) : ptr {arr}, len {N} {}

    template <typename U, std::size_t N>
    span(U (&arr)[N]) = delete;

    pointer data() const {
      return ptr;
    }

    std::size_t size() const {
      return len;
    }

    bool empty() const {
      return len == 0;
    }

    reference operator[](std::size_t index) const {
      return ptr[index];
    }

    iterator begin() const {
      return ptr;
    }

    iterator end() const {
      return ptr + len;
    }

  private:
    pointer ptr;
    std::size_t len;
  };
}
