#include "csv.h"

#include <fstream>
#include <iostream>
#include <sstream>

CSVData::CSVData(int num_colums, const std::string base_name)
{
    columns = num_colums;

    for (int i=0;i<columns;i++)
    {
        header +=base_name+"_"+std::to_string(i);
        if (i+1 != columns)
            header+=",";
    }
}

CSVData::CSVData(const std::vector<std::string> &column_names)
{
    columns = column_names.size();
    for (int i=0;i<columns;i++)
    {
        header += column_names[i];
        if (i+1 != columns)
            header+=",";
    }
}

void CSVData::add_row(std::vector<float> &values, float base_value)
{
    body.emplace_back();
    for (int i=0;i<std::min<int>(columns, values.size());i++)
    {
        body.back() += std::to_string(values[i]);
        if (i + 1 != columns)
            body.back() += ",";
    }
    for (int i=std::min<int>(columns, values.size());i<columns;i++)
    {
        body.back() += std::to_string(base_value);
        if (i + 1 != columns)
            body.back() += ",";        
    }
    rows++;
}

void CSVData::add_row(std::vector<double> &values, double base_value)
{
    body.emplace_back();
    for (int i=0;i<std::min<int>(columns, values.size());i++)
    {
        body.back() += std::to_string(values[i]);
        if (i + 1 != columns)
            body.back() += ",";
    }
    for (int i=std::min<int>(columns, values.size());i<columns;i++)
    {
        body.back() += std::to_string(base_value);
        if (i + 1 != columns)
            body.back() += ",";        
    }
    rows++;
}

void CSVData::add_row(std::vector<int> &values, int base_value)
{
    body.emplace_back();
    for (int i=0;i<std::min<int>(columns, values.size());i++)
    {
        body.back() += std::to_string(values[i]);
        if (i + 1 != columns)
            body.back() += ",";
    }
    for (int i=std::min<int>(columns, values.size());i<columns;i++)
    {
        body.back() += std::to_string(base_value);
        if (i + 1 != columns)
            body.back() += ",";        
    }
    rows++;
}

void CSVData::add_row(std::vector<std::string> &values, std::string base_value)
{
    body.emplace_back();
    for (int i=0;i<std::min<int>(columns, values.size());i++)
    {
        body.back() += values[i];
        if (i + 1 != columns)
            body.back() += ",";
    }
    for (int i=std::min<int>(columns, values.size());i<columns;i++)
    {
        body.back() += base_value;
        if (i + 1 != columns)
            body.back() += ",";        
    }
    rows++;
}

bool CSVSaver::save_csv_in_file(CSVData &data, std::string filename)
{
    if (data.columns == 0)
        return false;
    try
    {
    
        std::string input = data.header;

        for (auto &r : data.body)
        {
            input +="\n"+r;
        }

        std::ofstream out(filename);
        out << input;
        out.close();
        return true;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }

}