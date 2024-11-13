#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <utility>
#include <fstream>
#include <sstream>
#include <cassert>
#include <cctype>
#include <regex>

#include <LiteMath.h>

#include <stp_parser.hpp>

using namespace LiteMath;

namespace STEP {

Type str2type(std::string name) {
    if (name == "CARTESIAN_POINT")
        return Type::POINT;
    else if (name == "REPRESENTATION_ITEM")
        return Type::REPRESENTATION_ITEM;
    else if (name == "GEOMETRIC_REPRESENTATION_ITEM")
        return Type::GEOMETRIC_REPRESENTATION_ITEM;
    else if (name == "B_SPLINE_SURFACE_WITH_KNOTS")
        return Type::BSPLINE_SURFACE_WITH_KNOTS;
    else if (name == "RATIONAL_B_SPLINE_SURFACE")
        return Type::RATIONAL_BSPLINE_SURFACE;
    else if (name == "B_SPLINE_SURFACE")
        return Type::BSPLINE_SURFACE;
    else if (name == "BOUNDED_SURFACE")
        return Type::BOUNDED_SURFACE;
    else if (name == "SURFACE")
        return Type::SURFACE;
    else if (name == "COMPLEX")
        return Type::COMPLEX;
    return Type::UNDEFINED;
}

std::string type2str(Type type) {
    if (type == Type::POINT)
        return "CARTESIAN_POINT";
    else if (type == Type::REPRESENTATION_ITEM)
        return "REPRESENTATION_ITEM";
    else if (type == Type::GEOMETRIC_REPRESENTATION_ITEM)
        return "GEOMETRIC_REPRESENTATION_ITEM";
    else if (type == Type::BSPLINE_SURFACE_WITH_KNOTS)
        return "B_SPLINE_SURFACE_WITH_KNOTS";
    else if (type == Type::RATIONAL_BSPLINE_SURFACE)
        return "RATIONAL_B_SPLINE_SURFACE";
    else if (type == Type::BSPLINE_SURFACE)
        return "B_SPLINE_SURFACE"; 
    else if (type == Type::BOUNDED_SURFACE)
        return "BOUNDED_SURFACE";
    else if (type == Type::SURFACE)
        return "SURFACE";
    else if (type == Type::COMPLEX)
        return "COMPLEX";
    return "UNDEFINED";
}

std::vector<std::string> argsplit(const std::string &rawargs, bool bycomma) {
    int level = 0;
    std::vector<uint> argpos = { 1 };
    bool isname = true;
    for (uint idx = 0; idx < rawargs.size(); idx++) {
        char chr = rawargs[idx];
        if (chr == '(') level++;
        else if (chr == ')') level--;
        else if (bycomma && chr == ',' && level == 1) argpos.push_back(idx + 1);
        else if (!bycomma && level == 1 && !isname) { argpos.push_back(idx); isname = true; }
        if (level > 1) isname = false;
    }

    argpos.push_back(rawargs.size() - !bycomma);

    std::vector<std::string> args;
    for (uint idx = 1; idx < argpos.size(); idx++) {
        uint cur = argpos[idx - 1], next = argpos[idx];
        std::string arg = rawargs.substr(cur, next - cur - bycomma);
        args.push_back(arg);
    }

    return args;
}

std::string Parser::readEntry(const std::string &text, size_t &offset) {
    std::string entry;
    bool iscomment = false;
    size_t size = text.size();
    for (; offset < size - 1; offset++) {
        char prev = text[offset-1], chr = text[offset], next = text[offset+1];
        if (std::isspace(chr)) continue;
        if (!iscomment && chr == ';') { offset++; break; }

        if (chr == '/' && next == '*') iscomment = true;
        else if (prev == '*' && chr == '/') { iscomment = false; continue; }

        if (!iscomment) entry.push_back(chr);
    }
    return entry;
}

Parser::Parser(const std::string &filename) {
    std::ifstream file(filename);
    std::stringstream stream;

    // Add whitespaces for always accessing
    // previous, current and next chars in readEntry
    stream << ' ' << file.rdbuf() << ' ';

    std::string text = stream.str();
    file.close();

    bool isDataSection = false;
    std::string entry;
    size_t offset = 1;
    while (true) {
        entry = this->readEntry(text, offset);
        size_t size = entry.size();
        if (size == 0) break;

        if (entry == "DATA") { isDataSection = true; continue; }
        if (entry == "ENDSEC" && isDataSection) isDataSection = false;
        if (!isDataSection) continue;

        Entity entity = this->parseEntity(entry);
        this->entities[entity.id] = std::move(entity);
    }
}

uint Parser::parseID(std::string rawID) {
    std::regex rexp("#(\\d+)");
    std::smatch match;
    std::regex_match(rawID, match, rexp);
    uint id = std::stoi(match[1].str());
    return id;
}

uint Parser::parseF(std::string raw) {
    return std::stof(raw);
}

uint Parser::parseU(std::string raw) {
    return std::stoi(raw);
}

std::vector<uint> Parser::parseUVector1D(std::string raw) {
    std::vector<std::string> args = argsplit(raw);
    std::vector<uint> uvector1D;
    for (auto arg : args) {
        uint num = stoi(arg);
        uvector1D.push_back(num);
    }
    return uvector1D;
}

std::vector<float> Parser::parseFVector1D(std::string raw) {
    std::vector<std::string> args = argsplit(raw);
    std::vector<float> fvector1D;
    for (auto arg : args) {
        float num = stof(arg);
        fvector1D.push_back(num);
    }
    return fvector1D;
}

float3 Parser::tofloat3(uint id) {
    auto &entity = this->entities[id];
    assert(entity.type == Type::POINT);

    float3 point = float3();
    std::string coords_arg = entity.args[1];

    // Parse coords
    std::vector<std::string> coords = argsplit(coords_arg);
    assert(coords.size() == 3);
    for (size_t i = 0; i < coords.size(); i++) {
        point[i] = std::stof(coords[i]);
    }

    return point;
}

Vector2D<float> Parser::parseFVector2D(std::string raw) {
    std::vector<std::string> points_rows = argsplit(raw);
    size_t rows = points_rows.size();
    size_t cols = argsplit(points_rows[0]).size();
    Vector2D<float> points(rows, cols);
    for (size_t i = 0; i < rows; i++) {
        std::vector<std::string> points_row = argsplit(points_rows[i]);
        for (size_t j = 0; j < cols; j++) {
            std::string rawF = points_row[j];
            auto index = std::make_pair(i, j);
            points[index] = this->parseF(rawF);
        }
    }
    return points;
}

Vector2D<float4> Parser::parsePointVector2D(std::string raw) {
    std::vector<std::string> points_rows = argsplit(raw);
    size_t rows = points_rows.size();
    size_t cols = argsplit(points_rows[0]).size();
    Vector2D<float4> points(rows, cols);
    for (size_t i = 0; i < rows; i++) {
        std::vector<std::string> points_row = argsplit(points_rows[i]);
        for (size_t j = 0; j < cols; j++) {
            std::string rawID = points_row[j];
            uint pointID = this->parseID(rawID);
            float3 point = this->tofloat3(pointID);
            auto index = std::make_pair(i, j);
            points[index] = ::to_float4(point, 1.0f);
        }
    }
    return points;
}

Entity Parser::parseComplexArg(const std::string &arg) {
    // Returns complex entity name and args.
    // It is similar to simple STEP entity,
    // but since it is the argument, it doesn't have the id.
    // Although the types of complex arguments match the
    // STEP entity types, their arguments are just 'subarguments'
    // from the real STEP entity.
    // Therefore, their args must be parsed in a different way.
    std::regex rexp;
    std::smatch matches;
    Type type;
    std::vector<std::string> args;

    rexp = std::regex("(\\S+?)(\\(.*\\))");
    std::regex_search(arg, matches, rexp);

    std::string rawtype = matches[1].str();
    std::string rawargs = matches[2].str();
    type = str2type(rawtype);
    args = argsplit(rawargs);

    Entity entity;
    entity.id = 0;
    entity.type = type;
    entity.args = args;
    return entity;
}

Entity Parser::parseEntity(const std::string &entry) {
    std::string id;
    std::string type;
    std::string args;

    bool storedID   = false;
    bool storedName = false;
    for (auto chr : entry) {
        if (chr == '=') { storedID = true; continue; }
        if (!storedID && std::isdigit(chr)) { id.push_back(chr); continue; }

        if (chr == '(') storedName = true;
        if (!storedName && storedID) { type.push_back(chr); continue; }

        if (storedID && storedName) args.push_back(chr);
    }

    Entity entity;
    entity.id = this->parseU(id);

    if (type.size() == 0) {
        entity.type = Type::COMPLEX;
        entity.args = argsplit(args, false);
    }
    else {
        entity.type = str2type(type);
        entity.args = argsplit(args, true);
    }
    return entity;
}

Entity Parser::getEntity(uint id) {
    return this->entities[id];
}

/**************************************************************************/
/*************             NURBS section start                *************/
/**************************************************************************/
std::vector<float> decompressKnots(
        std::vector<float> &knots_comp,
        std::vector<uint> &knots_mult) {
    assert(knots_comp.size() == knots_mult.size());

    std::vector<float> knots;
    for (size_t i = 0; i < knots_comp.size(); i++) {
        for (size_t j = 0; j < knots_mult[i]; j++) {
            knots.push_back(knots_comp[i]);
        }
    }
    return knots;
}

void trimKnots(std::vector<float> &knots, const std::vector<uint> &knots_mult, uint degree) {
    // Convert first and last knots to 'nurbs' format,
    // so that they are repated p+1 times where p is corresponding degree.
    // This is a crutch, because there should be an option in STEP
    // that corresponds to the degree multiplicities type.
    if (knots_mult[0] == 1) {
        for (size_t i = 0; i < degree; i++) {
            knots[i] = knots[degree];
        }
    }

    size_t size = knots.size();
    if (knots_mult[size-1] == 1) {
         for (size_t i = 0; i < degree; i++) {
            knots[size-i-1] = knots[size-degree-1];
         }
    }
}

void Parser::storeBSplineSurface(Entity &entity, RawNURBS &nurbs) {
    // Store BSPLINE_SURFACE complex entity part to NURBS
    assert(entity.type == Type::BSPLINE_SURFACE);

    std::string u_degree_arg     = entity.args[0];
    std::string v_degree_arg     = entity.args[1];
    std::string points_arg       = entity.args[2];

    // Parse degrees
    nurbs.u_degree = this->parseU(u_degree_arg);
    nurbs.v_degree = this->parseU(v_degree_arg);

    // Parse control points
    nurbs.points = this->parsePointVector2D(points_arg);
}

void Parser::storeBSplineSurfaceWithKnots(Entity &entity, RawNURBS &nurbs) {
    // Store BSPLINE_SURFACE_WITH_KNOTS complex entity part to NURBS
    assert(entity.type == Type::BSPLINE_SURFACE_WITH_KNOTS);

    std::string u_knots_mult_arg = entity.args[0];
    std::string v_knots_mult_arg = entity.args[1];
    std::string u_knots_arg      = entity.args[2];
    std::string v_knots_arg      = entity.args[3];

    // Parse knot_multiplicities
    std::vector<uint> u_knots_mult = this->parseUVector1D(u_knots_mult_arg);
    std::vector<uint> v_knots_mult = this->parseUVector1D(v_knots_mult_arg);

    // Parse knots
    std::vector<float> u_knots_comp = this->parseFVector1D(u_knots_arg);
    std::vector<float> v_knots_comp = this->parseFVector1D(v_knots_arg);
    nurbs.u_knots = decompressKnots(u_knots_comp, u_knots_mult);
    nurbs.v_knots = decompressKnots(v_knots_comp, v_knots_mult);

    // Trim knots
    trimKnots(nurbs.u_knots, u_knots_mult, nurbs.u_degree);
    trimKnots(nurbs.v_knots, v_knots_mult, nurbs.v_degree);
}

void Parser::storeRationalBSplineSurface(Entity &entity, RawNURBS &nurbs) {
    assert(entity.type == Type::RATIONAL_BSPLINE_SURFACE);

    std::string weights_arg = entity.args[0];

    // Parse weights
    nurbs.weights = parseFVector2D(weights_arg);
}

RawNURBS Parser::BSplineSurfaceWithKnotsToNURBS(uint id) {
    Entity entity = this->getEntity(id);
    RawNURBS nurbs;

    std::string u_degree_arg     = entity.args[1];
    std::string v_degree_arg     = entity.args[2];
    std::string points_arg       = entity.args[3];
    std::string u_knots_mult_arg = entity.args[8];
    std::string v_knots_mult_arg = entity.args[9];
    std::string u_knots_arg      = entity.args[10];
    std::string v_knots_arg      = entity.args[11];

    // Parse degrees
    nurbs.u_degree = this->parseU(u_degree_arg);
    nurbs.v_degree = this->parseU(v_degree_arg);

    // Parse control points
    nurbs.points = this->parsePointVector2D(points_arg);

    // Parse knot_multiplicities
    std::vector<uint> u_knots_mult = this->parseUVector1D(u_knots_mult_arg);
    std::vector<uint> v_knots_mult = this->parseUVector1D(v_knots_mult_arg);

    // Parse knots
    std::vector<float> u_knots_comp = this->parseFVector1D(u_knots_arg);
    std::vector<float> v_knots_comp = this->parseFVector1D(v_knots_arg);
    nurbs.u_knots = decompressKnots(u_knots_comp, u_knots_mult);
    nurbs.v_knots = decompressKnots(v_knots_comp, v_knots_mult);

    // Trim knots
    trimKnots(nurbs.u_knots, u_knots_mult, nurbs.u_degree);
    trimKnots(nurbs.v_knots, v_knots_mult, nurbs.v_degree);

    // Make default weights
    size_t rows = nurbs.points.rows_count(), cols = nurbs.points.cols_count();
    nurbs.weights = Vector2D<float>(rows, cols);
    for (size_t i = 0; i < rows; i++)
        for (size_t j = 0; j < cols; j++) {
            auto index = std::make_pair(i, j);
            nurbs.weights[index] = 1;
        }

    return std::move(nurbs);
}

RawNURBS Parser::RationalBSplineSurfaceToNURBS(uint id) {
    Entity entity = this->getEntity(id);

    Entity BSplineSurface = this->parseComplexArg(entity.args[1]);
    Entity BSplineSurfaceWithKnots = this->parseComplexArg(entity.args[2]);
    Entity RationalBSplineSurface = this->parseComplexArg(entity.args[4]);

    RawNURBS nurbs;
    this->storeBSplineSurface(BSplineSurface, nurbs);
    this->storeBSplineSurfaceWithKnots(BSplineSurfaceWithKnots, nurbs);
    this->storeRationalBSplineSurface(RationalBSplineSurface, nurbs);
    return nurbs;
}

RawNURBS Parser::toNURBS(uint id) {
    // Call this function iff the nurbs is passed
    Entity entity = this->getEntity(id);
    if (isBSplineWithKnots(id)) return this->BSplineSurfaceWithKnotsToNURBS(id);
    else if (isRationalBSpline(id)) return this->RationalBSplineSurfaceToNURBS(id);

    assert(false);
    return RawNURBS();
}

bool Parser::isBSplineWithKnots(uint id) {
    Entity entity = this->getEntity(id);
    return entity.type == Type::BSPLINE_SURFACE_WITH_KNOTS;
}

bool Parser::isRationalBSpline(uint id) {
    Entity entity = this->getEntity(id);

    bool result = false;
    if (entity.type == Type::COMPLEX && entity.args.size() == 7) {
        result = this->parseComplexArg(entity.args[0]).type == Type::BOUNDED_SURFACE &&
            this->parseComplexArg(entity.args[1]).type == Type::BSPLINE_SURFACE &&
            this->parseComplexArg(entity.args[2]).type == Type::BSPLINE_SURFACE_WITH_KNOTS &&
            this->parseComplexArg(entity.args[3]).type == Type::GEOMETRIC_REPRESENTATION_ITEM &&
            this->parseComplexArg(entity.args[4]).type == Type::RATIONAL_BSPLINE_SURFACE &&
            this->parseComplexArg(entity.args[5]).type == Type::REPRESENTATION_ITEM &&
            this->parseComplexArg(entity.args[6]).type == Type::SURFACE;
    }
    return result;
}

bool Parser::isConvertableToNurbs(uint id) {
    return this->isBSplineWithKnots(id) || this->isRationalBSpline(id);
}

std::vector<RawNURBS> Parser::allNURBS() {
    std::vector<RawNURBS> allNurbs;
    for (auto &pair : this->entities) {
        auto id = pair.first;
        auto entity = pair.second;
        if (this->isConvertableToNurbs(id)) {
          allNurbs.push_back(this->toNURBS(id));
        }
    }
    return allNurbs;
}

std::map<uint, RawNURBS> Parser::allIDNurbs() {
    std::map<uint, RawNURBS> IDNurbs;
    for (auto &pair : this->entities) {
        auto id = pair.first;
        auto entity = pair.second;
        if (this->isConvertableToNurbs(id)) {
          IDNurbs[id] = this->toNURBS(id);
        }
    }
    return IDNurbs;
}

std::ostream& operator<<(std::ostream& cout, const STEP::RawNURBS &nurbs) {
    // Control points dimensions
    uint32_t n = nurbs.points.rows_count();
    cout << "n = " << n-1 << std::endl;

    uint32_t m = nurbs.points.cols_count();
    cout << "m = " << m-1 << std::endl;

    // Control points
    cout << "points:" << std::endl;
    for (size_t i = 0; i < n; i++) {
        for (size_t j = 0; j < m; j++) {
            auto index = std::make_pair(i, j);
            auto point = nurbs.points[index];
            cout << "{" << point.x << " " << point.z << " " << point.y << "}\t";
        }
        cout << std::endl;
    }

    // Weights
    cout << "weights:" << std::endl;
    for (size_t i = 0; i < n; i++) {
        for (size_t j = 0; j < m; j++) {
            auto index = std::make_pair(i, j);
            auto point = nurbs.points[index];
            cout << nurbs.weights[index] << " ";
        }
        cout << std::endl;
    }

    // Degrees
    cout << "u_degree: " << nurbs.u_degree << std::endl;
    cout << "v_degree: " << nurbs.v_degree << std::endl;

    // Knots
    cout << "u_knots: ";
    for (auto knot : nurbs.u_knots)
        cout << knot << " ";
    cout << std::endl;

    cout << "v_knots: ";
    for (auto knot : nurbs.v_knots)
        cout << knot << " ";
    cout << std::endl;

    return cout;
}
/**************************************************************************/
/*************              NURBS section end                 *************/
/**************************************************************************/

} // namespace STEP
