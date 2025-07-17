#pragma once
#include <optional>

template <typename T, typename E>
struct Result
{
    std::optional<T> value;
    E error;

    static Result<T, E> Ok(T val)
    {
        return {std::move(val), E{}};
    }

    static Result<T, E> Err(E err)
    {
        return {std::nullopt, err};
    }

    bool is_ok() const
    {
        return value.has_value();
    }

    const T &unwrap() const
    {
        return *value;
    }

    E error_code() const
    {
        return error;
    }
};