#pragma once
#include <vector>
#include <string>
#include "LiteMath.h"

using LiteMath::float2;
using LiteMath::float3;
using LiteMath::float3x3;
using LiteMath::float4;
using LiteMath::float4x4;
using LiteMath::int2;
using LiteMath::int3;
using LiteMath::int4;
using LiteMath::uint2;
using LiteMath::uint3;
using LiteMath::uint4;

struct Block;
class BlkManager;
struct Block
{
    struct DataArray;
    enum ValueType
    {
        EMPTY,
        BOOL,
        INT,
        UINT64,
        DOUBLE,
        VEC2,
        VEC3,
        VEC4,
        IVEC2,
        IVEC3,
        IVEC4,
        MAT4,
        STRING,
        BLOCK,
        ARRAY
    };

    struct Value
    {
        ValueType type;
        union
        {
            bool b;
            long i;
            uint64_t u;
            double d;
            float2 v2;
            float3 v3;
            float4 v4;
            int2 iv2;
            int3 iv3;
            int4 iv4;
            float4x4 m4;
            std::string *s;
            Block *bl;
            DataArray *a;
        };
        Value()
        {
            type = EMPTY;
        }
        Value(const Value &v)
        {
            type = v.type;
            if (type == ValueType::INT)
                i = v.i;
            if (type == ValueType::UINT64)
                u = v.u;
            else if (type == ValueType::BOOL)
                b = v.b;
            else if (type == ValueType::DOUBLE)
                d = v.d;
            else if (type == ValueType::VEC2)
                v2 = v.v2;
            else if (type == ValueType::VEC3)
                v3 = v.v3;
            else if (type == ValueType::VEC4)
                v4 = v.v4;
            else if (type == ValueType::IVEC2)
                iv2 = v.iv2;
            else if (type == ValueType::IVEC3)
                iv3 = v.iv3;
            else if (type == ValueType::IVEC4)
                iv4 = v.iv4;
            else if (type == ValueType::MAT4)
                m4 = v.m4;
            else if (type == ValueType::STRING)
                s = v.s;
            else if (type == ValueType::BLOCK)
                bl = v.bl;
            else if (type == ValueType::ARRAY)
                a = v.a;
        }
        void copy(const Value &v)
        {
            clear();
            type = v.type;
            if (type == ValueType::INT)
                i = v.i;
            if (type == ValueType::UINT64)
                u = v.u;
            else if (type == ValueType::BOOL)
                b = v.b;
            else if (type == ValueType::DOUBLE)
                d = v.d;
            else if (type == ValueType::VEC2)
                v2 = v.v2;
            else if (type == ValueType::VEC3)
                v3 = v.v3;
            else if (type == ValueType::VEC4)
                v4 = v.v4;
            else if (type == ValueType::IVEC2)
                iv2 = v.iv2;
            else if (type == ValueType::IVEC3)
                iv3 = v.iv3;
            else if (type == ValueType::IVEC4)
                iv4 = v.iv4;
            else if (type == ValueType::MAT4)
                m4 = v.m4;
            else if (type == ValueType::STRING)
            {
                s = new std::string();
                if (v.s)
                    *s = *v.s;
            }
            else if (type == ValueType::BLOCK)
            {
                bl = new Block();
                if (v.bl)
                    bl->copy(v.bl);
            }
            else if (type == ValueType::ARRAY)
            {
                a = new DataArray();
                if (v.a)
                    *a = *v.a;
            }
        }
        ~Value()
        {

        }
        void clear();
    };

    struct DataArray
    {
        ValueType type = EMPTY;
        std::vector<Value> values;
    };
    friend class BlkManager;

    int size() const;
    void clear();
    void copy(const Block *b);
    ~Block();
    Block& operator=(Block &b);
    Block& operator=(Block &&b);
    bool has_tag(const std::string &name) const;
    int get_id(const std::string &name) const;
    int get_next_id(const std::string &name, int pos) const;
    std::string get_name(int id) const;
    ValueType get_type(int id) const;
    ValueType get_type(const std::string &name) const;

    int get_bool(int id, bool base_val = false) const;
    int get_int(int id, int base_val = 0) const;
    uint64_t get_uint64(int id, uint64_t base_val = 0) const;
    double get_double(int id, double base_val = 0) const;
    float2 get_vec2(int id, float2 base_val = float2(0, 0)) const;
    float3 get_vec3(int id, float3 base_val = float3(0, 0, 0)) const;
    float4 get_vec4(int id, float4 base_val = float4(0, 0, 0, 0)) const;
    int2 get_ivec2(int id, int2 base_val = int2(0, 0)) const;
    int3 get_ivec3(int id, int3 base_val = int3(0, 0, 0)) const;
    int4 get_ivec4(int id, int4 base_val = int4(0, 0, 0, 0)) const;
    float4x4 get_mat4(int id, float4x4 base_val = float4x4()) const;
    std::string get_string(int id, std::string base_val = "") const;
    Block *get_block(int id) const;
    bool get_arr(int id, std::vector<double> &values, bool replace = false) const;
    bool get_arr(int id, std::vector<float> &values, bool replace = false) const;
    bool get_arr(int id, std::vector<int> &values, bool replace = false) const;
    bool get_arr(int id, std::vector<unsigned> &values, bool replace = false) const;
    bool get_arr(int id, std::vector<short> &values, bool replace = false) const;
    bool get_arr(int id, std::vector<unsigned short> &values, bool replace = false) const;
    bool get_arr(int id, std::vector<std::string> &values, bool replace = false) const;

    int get_bool(const std::string name, bool base_val = false) const;
    int get_int(const std::string name, int base_val = 0) const;
    uint64_t get_uint64(const std::string name, uint64_t base_val = 0) const;
    double get_double(const std::string name, double base_val = 0) const;
    float2 get_vec2(const std::string name, float2 base_val = float2(0, 0)) const;
    float3 get_vec3(const std::string name, float3 base_val = float3(0, 0, 0)) const;
    float4 get_vec4(const std::string name, float4 base_val = float4(0, 0, 0, 0)) const;
    int2 get_ivec2(const std::string name, int2 base_val = int2(0, 0)) const;
    int3 get_ivec3(const std::string name, int3 base_val = int3(0, 0, 0)) const;
    int4 get_ivec4(const std::string name, int4 base_val = int4(0, 0, 0, 0)) const;
    float4x4 get_mat4(const std::string name, float4x4 base_val = float4x4()) const;
    std::string get_string(const std::string name, std::string base_val = "") const;
    Block *get_block(std::string name) const;
    bool get_arr(const std::string name, std::vector<double> &values, bool replace = false) const;
    bool get_arr(const std::string name, std::vector<float> &values, bool replace = false) const;
    bool get_arr(const std::string name, std::vector<int> &values, bool replace = false) const;
    bool get_arr(const std::string name, std::vector<unsigned> &values, bool replace = false) const;
    bool get_arr(const std::string name, std::vector<short> &values, bool replace = false) const;
    bool get_arr(const std::string name, std::vector<unsigned short> &values, bool replace = false) const;
    bool get_arr(const std::string name, std::vector<std::string> &values, bool replace = false) const;

    void add_bool(const std::string name, bool base_val = false);
    void add_int(const std::string name, int base_val = 0);
    void add_uint64(const std::string name, uint64_t base_val = 0);
    void add_double(const std::string name, double base_val = 0);
    void add_vec2(const std::string name, float2 base_val = float2(0, 0));
    void add_vec3(const std::string name, float3 base_val = float3(0, 0, 0));
    void add_vec4(const std::string name, float4 base_val = float4(0, 0, 0, 0));
    void add_ivec2(const std::string name, int2 base_val = int2(0, 0));
    void add_ivec3(const std::string name, int3 base_val = int3(0, 0, 0));
    void add_ivec4(const std::string name, int4 base_val = int4(0, 0, 0, 0));
    void add_mat4(const std::string name, float4x4 base_val = float4x4());
    void add_string(const std::string name, std::string base_val = "");
    void add_block(const std::string name, Block *bl);
    void add_arr(const std::string name, std::vector<double> &values);
    void add_arr(const std::string name, std::vector<float> &values);
    void add_arr(const std::string name, std::vector<int> &values);
    void add_arr(const std::string name, std::vector<unsigned> &values);
    void add_arr(const std::string name, std::vector<short> &values);
    void add_arr(const std::string name, std::vector<unsigned short> &values);
    void add_arr(const std::string name, std::vector<std::string> &values);

    void set_bool(const std::string name, bool base_val = false);
    void set_int(const std::string name, int base_val = 0);
    void set_uint64(const std::string name, uint64_t base_val = 0);
    void set_double(const std::string name, double base_val = 0);
    void set_vec2(const std::string name, float2 base_val = float2(0, 0));
    void set_vec3(const std::string name, float3 base_val = float3(0, 0, 0));
    void set_vec4(const std::string name, float4 base_val = float4(0, 0, 0, 0));
    void set_ivec2(const std::string name, int2 base_val = int2(0, 0));
    void set_ivec3(const std::string name, int3 base_val = int3(0, 0, 0));
    void set_ivec4(const std::string name, int4 base_val = int4(0, 0, 0, 0));
    void set_mat4(const std::string name, float4x4 base_val = float4x4());
    void set_string(const std::string name, std::string base_val = "");
    void set_block(const std::string name, Block *bl);
    void set_arr(const std::string name, const std::vector<double> &values);
    void set_arr(const std::string name, const std::vector<float> &values);
    void set_arr(const std::string name, const std::vector<int> &values);
    void set_arr(const std::string name, const std::vector<unsigned> &values);
    void set_arr(const std::string name, const std::vector<short> &values);
    void set_arr(const std::string name, const std::vector<unsigned short> &values);
    void set_arr(const std::string name, const std::vector<std::string> &values);

    void add_value(const std::string &name, const Value &value);
    void set_value(const std::string &name, const Value &value);

    void add_detalization(Block &det);

    std::vector<std::string> names;
    std::vector<Value> values;
};

extern bool load_block_from_string(std::string &str, Block &b);
extern bool load_block_from_file(std::string path, Block &b);
extern void save_block_to_string(std::string &str, Block &b);
extern void save_block_to_file(std::string path, Block &b);
extern std::string base_blk_path;