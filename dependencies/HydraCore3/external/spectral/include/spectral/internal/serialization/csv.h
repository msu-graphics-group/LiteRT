#ifndef INCLUDE_SPECTRAL_INTERNAL_SERIALIZATION_CSV_H
#define INCLUDE_SPECTRAL_INTERNAL_SERIALIZATION_CSV_H
#include <spectral/internal/serialization/parsers.h>
#include <unordered_map>
#include <vector>
#include <istream>
#include <tuple>
#include <string>
#include <limits>
#include <optional>


namespace spec {

    namespace csv {
        struct skip {};
    }

    template<>
    struct Parser<csv::skip>
    {
        static csv::skip parse(const std::string &)
        {
            return {};
        }
    };

    namespace csv {
        constexpr char COMMENT_SYM = '#';

        /**
         *  Returns empty optional on empty input or on comments.
         * Otherwise throws exception
         */

        template<typename Head>
        std::optional<std::tuple<Head>> _parse_line(const std::string &str, char delim, size_t &pos, bool ignore_comments)
        {
            if(pos == std::string::npos || (ignore_comments && str[pos] == COMMENT_SYM)) return {};
            size_t nextpos = str.find(delim, pos);
            std::string word = str.substr(pos, nextpos);

            pos = nextpos + (nextpos != std::string::npos);
            return {parse<Head>(word)};
        }

        template<typename Head, typename... Columns>
        std::enable_if_t<(sizeof...(Columns) > 0), std::optional<std::tuple<Head, Columns...>>> 
        _parse_line(const std::string &str, char delim, size_t &pos, bool ignore_comments)
        {
            if(pos == std::string::npos || (ignore_comments && str[pos] == COMMENT_SYM)) return {};
            size_t nextpos = str.find(delim, pos);
            std::string word = str.substr(pos, nextpos);

            std::tuple<Head> t1{parse<Head>(word)};

            pos = nextpos + (nextpos != std::string::npos);
            std::optional<std::tuple<Columns...>> t2 = _parse_line<Columns...>(str, delim, pos, ignore_comments);
            if(!t2) throw std::runtime_error("Unfinished csv line");
            return std::tuple_cat<>(t1, *t2);
        }

        template<typename Head, typename... Columns>
        std::optional<std::tuple<Head, Columns...>> parse_line(const std::string &str, char delim = ',', size_t pos = 0, bool ignore_comments = true)
        {
            return _parse_line<Head, Columns...>(str, delim, pos, ignore_comments);
        }

        template<typename Head, typename... Columns>
        std::optional<std::tuple<Head, Columns...>> parse_line(std::istream &stream, char delim = ',', bool ignore_comments = true)
        {
            std::string str;
            std::getline(stream, str);
            return parse_line<Head, Columns...>(str, delim, 0, ignore_comments);
        }


        template<typename Head, typename... Columns>
        std::vector<std::tuple<Head, Columns...>> load_as_vector(std::istream &stream, char delim = ',', int skip = 0, bool ignore_comments = true)
        {  
            for(; skip > 0; --skip) stream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::vector<std::tuple<Head, Columns...>> res;
            while(stream.peek() != EOF) {
                auto data = parse_line<Head, Columns...>(stream, delim, ignore_comments);
                if(data) {
                    res.push_back(*data);
                }
            }
            return res;
        }

        // Multicolumn versions

        template<typename T>
        std::optional<std::tuple<std::vector<T>>> parse_line_m(const std::string &str, char delim = ',', size_t pos = 0, bool ignore_comments = true)
        {
            std::vector<T> vec;
            while(pos != std::string::npos && (!ignore_comments || str[pos] != COMMENT_SYM)) {

                size_t nextpos = str.find(delim, pos);

                std::string word = str.substr(pos, nextpos);

                vec.push_back(parse<T>(word));
                pos = nextpos + (nextpos != std::string::npos);
            }
            return {vec};
        }

        template<typename T, typename... Columns, typename = std::enable_if_t<(sizeof...(Columns) > 0), void>>
        std::optional<std::tuple<std::vector<T>, Columns...>> parse_line_m(const std::string &str, char delim = ',', size_t pos = 0, bool ignore_comments = true)
        {
            //std::cout << str << std::endl;
            const auto pre = _parse_line<Columns...>(str, delim, pos, ignore_comments);
            if(!pre) throw std::runtime_error("No data found");

            const auto post = parse_line_m<T>(str, delim, pos, ignore_comments);
            return std::tuple_cat<>(*post, *pre);
        }

        template<typename T, typename... Columns>
        std::optional<std::tuple<std::vector<T>, Columns...>> parse_line_m(std::istream &stream, char delim = ',', bool ignore_comments = true)
        {
            std::string str;
            std::getline(stream, str);
            return parse_line_m<T, Columns...>(str, delim, 0, ignore_comments);
        }


        template<typename T, typename... Columns>
        std::vector<std::tuple<std::vector<T>, Columns...>> load_as_vector_m(std::istream &stream, char delim = ',', int skip = 0, bool ignore_comments = true)
        {  
            for(; skip > 0; --skip) stream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::vector<std::tuple<std::vector<T>, Columns...>> res;
            while(stream.peek() != EOF) {
                auto data = parse_line_m<T, Columns...>(stream, delim, ignore_comments);
                if(data) {
                    res.push_back(*data);
                }
            }
            return res;
        }

    }
}

#endif