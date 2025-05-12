#include <internal/serialization/envi.h>
#include <internal/serialization/parsers.h>
#include <istream>
#include <fstream>
#include <algorithm>
#include <cctype>
#include <unordered_map>
#include <stdexcept>

#include <cstring>
#include <iostream>

namespace spec {

    namespace {

        std::string ltrim(const std::string &str)
        {
            for(unsigned i = 0; i < str.size(); ++i) {
                if(!std::isspace(str[i])) {
                    return str.substr(i);
                }
            }
            return "";
        }

        std::string rtrim(const std::string &str)
        {
            for(int i = str.size() - 1; i >= 0; --i) {
                if(!std::isspace(str[i])) {
                    return str.substr(0, i + 1);
                }
            }
            return "";
        }

        std::string trim(const std::string &str)
        {
            return rtrim(ltrim(str));
        }

        std::string to_lower(std::string str) {
            std::transform(str.begin(), str.end(), str.begin(), tolower);
            return str;
        }

        std::string tgetline(std::istream &str)
        {
            std::string line;
            std::getline(str, line);
            return trim(line);
        }

        inline bool get(std::istream &str, int &c) {
            c = str.get();
            return !str.eof();
        }

        inline void skip_spaces(std::istream &str) {
            int c;
            while((c = str.peek(), !str.eof()) && isspace(c)) str.get();
        }

        void fill_entries(const std::string &path, std::unordered_map<std::string, std::string> &entries)
        {
            std::ifstream file{path};
            std::string line = tgetline(file);
            if(line != "ENVI") throw std::runtime_error("No ENVI string");

            while(file.peek(), !file.eof()) {
                std::string var_name{};
                std::string var_content{};
                bool is_block = false;
                int block_level = 0;
                bool found_eq = false;
                int c;
                skip_spaces(file);

                while(get(file, c)) {
                    if(found_eq) {
                        switch(c) {
                        case '{':
                            block_level += 1;
                            if(block_level != 1) {
                                var_content += c;
                            }
                            is_block = true;
                            break;
                        case '}':
                            if(block_level == 0) {
                                throw std::runtime_error("Unbalanced braces");
                            }
                            if(block_level != 1) {
                                var_content += c;
                            }
                            block_level -= 1;
                            if(block_level == 0) {
                                goto out;
                            }
                            break;
                        case '\n':
                            if(is_block) {
                                var_content += c;
                                break;
                            }
                            else {
                                goto out;
                            }
                        default:
                            var_content += c;
                        }
                    }
                    else {
                        if(c == '=') {
                            found_eq = true;
                            var_name = trim(var_name);
                            if(var_name.empty()) throw std::runtime_error("No variable specifier");
                        }
                        else {
                            var_name += c;
                        }
                    }
                }
                if(file.eof() && !var_name.empty()) {
                    throw std::runtime_error("Incorrect format");
                }
out:
                var_content = trim(var_content);
                if(is_block) {
                    if(block_level != 0) throw std::runtime_error("Unclosed braces");
                    var_content.pop_back();
                }
                entries.insert({var_name, var_content});
            }

        }

        template<typename T, typename P = T>
        void parse_simple_array(std::vector<T> &vec, std::string str, char delim)
        {
            str.erase(std::remove_if(str.begin(), str.end(), isspace), str.end());
            size_t pos = 0ull;
            size_t nextpos;
            do {
                nextpos = str.find(delim, pos);
                std::string word = str.substr(pos, nextpos);
                pos = nextpos + 1;
                vec.push_back(parse<P>(word));
            } while(nextpos != std::string::npos);

            vec.shrink_to_fit();
        }

    }

    using std::isspace;


    MetaENVI MetaENVI::load(const std::string &path)
    {
        MetaENVI meta;
        std::unordered_map<std::string, std::string> &entries = meta.additional;
        fill_entries(path, entries);

        decltype(meta.additional)::const_iterator it;

        if((it = entries.find("header offset")) != entries.end()) {
            meta.header_offset = parse<std::size_t>(it->second);
            entries.erase(it);
        }

        if((it = entries.find("wavelenghts units")) != entries.end()) {
            if(it->second == "nm" || it->second == "Nanometers") {
                meta.wavelength_units = UnitType::NANOMETER;
            }
            else meta.wavelength_units = UnitType::UNSUPPORTED;
            entries.erase(it);
        }

        if((it = entries.find("samples")) != entries.end()) {
            meta.samples = parse<int>(it->second);
            entries.erase(it);
        }

        if((it = entries.find("lines")) != entries.end()) {
            meta.lines = parse<int>(it->second);
            entries.erase(it);
        }

        if((it = entries.find("bands")) != entries.end()) {
            meta.bands = parse<int>(it->second);
            entries.erase(it);
        }

        if((it = entries.find("byte order")) != entries.end()) {
            meta.byte_order = parse<int>(it->second) == 0 ? ByteOrder::LITTLE_ENDIAN_ORDER : ByteOrder::BIG_ENDIAN_ORDER;
            entries.erase(it);
        }

        if((it = entries.find("data type")) != entries.end()) {
            switch(parse<int>(it->second)) {
            case 4:
                meta.data_type = DataType::FLOAT32;
                break;
            case 5:
                meta.data_type = DataType::FLOAT64;
                break;
            default:
                meta.data_type = DataType::UNSUPPORTED;
            }
            entries.erase(it);
        }

        if((it = entries.find("interleave")) != entries.end()) {
            std::string lower = to_lower(it->second);
            if(lower == "bsq") {
                meta.interleave = Interleave::BSQ;
            }
            else if(lower == "bil") {
                meta.interleave = Interleave::BIL;
            }
            else if(lower == "bip") {
                meta.interleave = Interleave::BIP;
            }
            else throw std::runtime_error("Unknown interleave type");
            entries.erase(it);
        }


        if((it = entries.find("wavelength")) != entries.end()) {
            if(meta.data_type == DataType::FLOAT32 || meta.data_type == DataType::FLOAT64) {
                parse_simple_array<Float>(meta.wavelength, it->second, ',');
            }
            else {
                throw std::runtime_error("Unsupported format");
            }
            entries.erase("wavelength");
        }
        else {
            throw std::runtime_error("No wavelength field found");
        }


        if((it = entries.find("Illuminant")) != entries.end()) {
            parse_simple_array<unsigned>(meta.illuminant, it->second, ';');
            entries.erase(it);
        }
        else {
            throw std::runtime_error("No Illuminant field found");
        }

        return meta;
    }
}