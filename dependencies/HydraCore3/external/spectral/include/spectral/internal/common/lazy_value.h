#ifndef INCLUDE_SPECTRAL_INTERNAL_COMMON_LAZY_VALUE_H
#define INCLUDE_SPECTRAL_INTERNAL_COMMON_LAZY_VALUE_H
#include <optional>
#include <functional>
#include <stdexcept>

template<typename T>
class LazyValue
{
public:
    template<typename P>
    LazyValue(const P &init)
        : constructor{[init]() -> T { return init(); }}, value{} {}

    LazyValue(LazyValue &&other)
        : constructor{}, value(std::move(other.value)) {}

    LazyValue(const LazyValue &) = delete;

    LazyValue &operator=(const LazyValue &) = delete;

    const T &get() const
    {
        if(!value) {
            value.emplace(std::move(constructor()));
        }
        return *value;
    }

    T &get() 
    {
        return const_cast<T &>(const_cast<const LazyValue<T> *>(this)->get());
    }

    const T &operator*() const
    {
        return get();
    }

    const T *operator->() const
    {
        return &get();
    }

    T &operator*()
    {
        return get();
    }

    T *operator->()
    {
        return &get();
    }

    operator bool() const
    {
        return bool(value);
    }

private:
    const std::function<T()> constructor;
    mutable std::optional<T> value;
};

template<typename T, typename P, typename = std::enable_if_t<!std::is_same<T, P>::value, void>>
LazyValue<T> make_lazy(const P &constructor)
{
    return LazyValue<T>(constructor);
}


template<typename T>
class LazyPtr
{
public:
    template<typename P>
    LazyPtr(const P &init)
        : constructor([init]() -> T *{ return init(); }) {}

    LazyPtr(LazyPtr &&other)
        : constructor(), value()
    {
        std::swap(value, other.value);
    }

    LazyPtr(const LazyPtr &) = delete;

    LazyPtr &operator=(const LazyPtr &) = delete;
    LazyPtr &operator=(const LazyPtr &&other)
    {
        std::swap(value, other.value);
    }

    const T *get() const
    {
        if(!value) {
            value = std::shared_ptr<T>(constructor());
            if(!value) throw std::runtime_error("Incorrect constructor in LazyPtr");
        }
        return value.get();
    }

    T *get()
    {
        return const_cast<T *>(const_cast<const LazyValue<T> *>(this)->get());
    }

    std::shared_ptr<T> get_shared() const
    {
        get();
        return value;
    }

    const T &operator*() const
    {
        return *get();
    }

    const T *operator->() const
    {
        return get();
    }

    T &operator*()
    {
        return *get();
    }

    T *operator->()
    {
        return get();
    }

private:
    const std::function<T *()> constructor;
    mutable std::shared_ptr<T> value;
};

template<typename T, typename P, typename = std::enable_if_t<!std::is_same<T, P>::value, void>>
LazyPtr<T> make_lazy_ptr(const P &constructor)
{
    return LazyPtr<T>(constructor);
}



#endif