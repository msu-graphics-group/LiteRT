#ifndef INCLUDE_SPECTRAL_INTERNAL_COMMON_HOLDER_H
#define INCLUDE_SPECTRAL_INTERNAL_COMMON_HOLDER_H
#include <spectral/internal/common/lazy_value.h>
#include <memory>

namespace spec {

    template<typename T>
    class ResourceHolder
    {
    public:
        ResourceHolder(std::shared_ptr<T> &&ptr) : object(std::shared_ptr<>(std::move(ptr))) {}
        ResourceHolder(LazyPtr<T> &&ptr) : object(LazyPtr<>(std::move(ptr))) {}
        ResourceHolder(const LazyPtr<T> &ptr) : object(ptr.get_shared()) {}
        ResourceHolder(const ResourceHolder &other) : object(copy_data(other)) {}

        ResourceHolder &operator=(const ResourceHolder &)
        {
            object = copy_data(other);
            return *this;
        }

        ResourceHolder(const ResourceHolder &&other) 
            : object() 
        {
            *this = other;
        }

        ResourceHolder &operator=(ResourceHolder &&other)
        {
            std::swap(object, other.object);
            return *this;
        }

        const T *get() const
        {
            if(std::holds_alternative<std::shared_ptr<T>>(object)) {
                return std::get<std::shared_ptr<T>>(object).get();
            }
            else {
                return std::get<LazyPtr<T>>(object).get();
            }
        }

        T *get() 
        {
            return const_cast<T *>(get());
        }

        const T &operator*() const
        {
            return *get();
        }

        T &operator*()
        {
            return *get();
        }

        const T *operator->() const
        {
            return get();
        }

        T *operator->()
        {
            return get();

    private:
        std::variant<std::shared_ptr<T>, LazyPtr<T>> object;

        static std::shared_ptr<T> copy_data(const ResourceHolder &r)
        {
            if(std::holds_alternative<std::shared_ptr<T>>(r.object)) {
                return std::get<LazyPtr<T>>(object).get_shared();
            }
            return std::get<std::shared_ptr<T>>(object);
        }
    };

    template<typename T, typename P>
    ResourceHolder lazy_resource(const P &constructor)
    {
        return ResourceHolder(LazyPtr<T>(P));
    }

    template<typename T>
    ResourceHolder resource(T *ptr)
    {
        return ResourceHolder(std::shared_ptr<T>(ptr));
    }

    template<typename T, typename ...Args>
    ResourceHolder make_resource(Args ...args)
    {
        return ResourceHolder(std::make_shared<T>(args));
    }

}

#endif