#ifndef INCLUDE_SPECTRAL_INTERNAL_COMMON_REFL_H
#define INCLUDE_SPECTRAL_INTERNAL_COMMON_REFL_H

namespace spec {

    class UniqueId
    {
    public:
        using IdType = unsigned;

        UniqueId() 
            : id(last_id++) {}
        UniqueId(const UniqueId &other)
            : id(other.id) {}

        IdType get() const 
        {
            return id;
        }

        operator IdType() const
        {
            return id;
        }

        bool operator==(const UniqueId &other) const
        {
            return other.id == id;
        }

    private:
        const IdType id;
        static IdType last_id;
    };

    struct Object {
        
    };

    class ReflectiveClass
    {
    public:
        inline static const UniqueId class_id{};
        const UniqueId &id() const { return class_id; }
    };

    template<typename T, typename P>
    inline bool isa(const P *obj)
    {
        return obj->class_id() == T::refl_class_id;
    }

    template<typename T, typename P>
    inline bool isa(const P &obj)
    {
        return obj.class_id() == T::refl_class_id;
    }

    template<typename T, typename P>
    T *dyn_cast(const P *obj)
    {
        if(isa<T, P>(obj)) {
            return static_cast<const T *>(obj);
        }
        return nullptr;
    }

    template<typename T, typename P>
    T *dyn_cast(P *obj)
    {
        if(isa<T, P>(obj)) {
            return static_cast<T *>(obj);
        }
        return nullptr;
    }

}

#define INJECT_ABSTRACT_REFL(...)\
    virtual const spec::UniqueId &class_id() const = 0;

#define INJECT_REFL(...)\
    inline static const spec::UniqueId refl_class_id{};\
    const spec::UniqueId &class_id() const { return refl_class_id; }

#endif