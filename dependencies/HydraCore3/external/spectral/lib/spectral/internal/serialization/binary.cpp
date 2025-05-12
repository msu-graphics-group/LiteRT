#include <internal/serialization/binary.h>
#include <algorithm>

namespace spec::binary {

    template void write(std::ostream &dst, Float val);
    template Float read(std::istream &src);

    template void write_vec(std::ostream &dst, const vec3 &val);
    template vec3 read_vec(std::istream &src);
}