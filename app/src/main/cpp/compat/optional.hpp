// 简化版的 std::optional 实现，避免使用引用限定符
#pragma once
#ifndef COMPAT_OPTIONAL_HPP
#define COMPAT_OPTIONAL_HPP

#include <utility>
#include <type_traits>
#include <stdexcept>

namespace std {

struct nullopt_t {
    explicit constexpr nullopt_t(int) {}
};
constexpr nullopt_t nullopt{0};

template<class T>
class optional {
private:
    bool has_value_;
    typename std::aligned_storage<sizeof(T), alignof(T)>::type storage_;

    void destroy() {
        if (has_value_) {
            reinterpret_cast<T*>(&storage_)->~T();
            has_value_ = false;
        }
    }

public:
    // Constructors
    constexpr optional() noexcept : has_value_(false), storage_() {}
    constexpr optional(nullopt_t) noexcept : has_value_(false), storage_() {}

    optional(const optional& other) : has_value_(other.has_value_) {
        if (other.has_value_) {
            new (&storage_) T(*reinterpret_cast<const T*>(&other.storage_));
        }
    }

    optional(optional&& other) noexcept(std::is_nothrow_move_constructible<T>::value)
        : has_value_(other.has_value_) {
        if (other.has_value_) {
            new (&storage_) T(std::move(*reinterpret_cast<T*>(&other.storage_)));
        }
    }

    template<class... Args>
    explicit optional(Args&&... args) : has_value_(true) {
        new (&storage_) T(std::forward<Args>(args)...);
    }

    // Destructor
    ~optional() {
        destroy();
    }

    // Assignment operators
    optional& operator=(nullopt_t) noexcept {
        destroy();
        return *this;
    }

    optional& operator=(const optional& other) {
        if (has_value_ && other.has_value_) {
            *reinterpret_cast<T*>(&storage_) = *reinterpret_cast<const T*>(&other.storage_);
        } else if (other.has_value_) {
            new (&storage_) T(*reinterpret_cast<const T*>(&other.storage_));
            has_value_ = true;
        } else {
            destroy();
        }
        return *this;
    }

    optional& operator=(optional&& other)
        noexcept(std::is_nothrow_move_assignable<T>::value &&
                std::is_nothrow_move_constructible<T>::value) {
        if (has_value_ && other.has_value_) {
            *reinterpret_cast<T*>(&storage_) = std::move(*reinterpret_cast<T*>(&other.storage_));
        } else if (other.has_value_) {
            new (&storage_) T(std::move(*reinterpret_cast<T*>(&other.storage_)));
            has_value_ = true;
        } else {
            destroy();
        }
        return *this;
    }

    // Observers
    constexpr explicit operator bool() const noexcept {
        return has_value_;
    }
    constexpr bool has_value() const noexcept {
        return has_value_;
    }

    const T& value() const {
        if (!has_value_) {
            throw std::runtime_error("Bad optional access");
        }
        return *reinterpret_cast<const T*>(&storage_);
    }

    T& value() {
        if (!has_value_) {
            throw std::runtime_error("Bad optional access");
        }
        return *reinterpret_cast<T*>(&storage_);
    }

    template<class U>
    constexpr T value_or(U&& default_value) const {
        return has_value_ ? *reinterpret_cast<const T*>(&storage_) : static_cast<T>(std::forward<U>(default_value));
    }

    template<class U>
    T value_or(U&& default_value) {
        return has_value_ ? *reinterpret_cast<T*>(&storage_) : static_cast<T>(std::forward<U>(default_value));
    }

    // Modifiers
    void reset() noexcept {
        destroy();
    }

    template<class... Args>
    void emplace(Args&&... args) {
        destroy();
        new (&storage_) T(std::forward<Args>(args)...);
        has_value_ = true;
    }

    // Comparison operators
    friend bool operator==(const optional& lhs, const optional& rhs) {
        if (lhs.has_value_ != rhs.has_value_) return false;
        if (!lhs.has_value_) return true;
        return *reinterpret_cast<const T*>(&lhs.storage_) == *reinterpret_cast<const T*>(&rhs.storage_);
    }

    friend bool operator!=(const optional& lhs, const optional& rhs) {
        return !(lhs == rhs);
    }
};

// 与 nullopt 比较的运算符
template<class T>
bool operator==(const optional<T>& opt, nullopt_t) noexcept {
    return !opt.has_value();
}

template<class T>
bool operator==(nullopt_t, const optional<T>& opt) noexcept {
    return !opt.has_value();
}

template<class T>
bool operator!=(const optional<T>& opt, nullopt_t) noexcept {
    return opt.has_value();
}

template<class T>
bool operator!=(nullopt_t, const optional<T>& opt) noexcept {
    return opt.has_value();
}

// 与值比较的运算符
template<class T, class U>
bool operator==(const optional<T>& opt, const U& value) {
    return opt.has_value() && opt.value() == value;
}

template<class T, class U>
bool operator==(const U& value, const optional<T>& opt) {
    return opt.has_value() && value == opt.value();
}

template<class T, class U>
bool operator!=(const optional<T>& opt, const U& value) {
    return !opt.has_value() || opt.value() != value;
}

template<class T, class U>
bool operator!=(const U& value, const optional<T>& opt) {
    return !opt.has_value() || value != opt.value();
}

} // namespace std

#endif // COMPAT_OPTIONAL_HPP