#pragma once
#include <vector>
#include <string>

struct CSVData
{
    std::string header;
    std::vector<std::string> body;
    int columns = 0;
    int rows = 0;
    CSVData();
    explicit CSVData(int num_colums, const std::string base_name = "column");
    CSVData(const std::vector<std::string> &columns);
    void add_row(std::vector<float> &values, float base_value = 0);
    void add_row(std::vector<double> &values, double base_value = 0);
    void add_row(std::vector<int> &values, int base_value = 0);
    void add_row(std::vector<std::string> &values, std::string base_value = "NAN");
};
class CSVSaver
{
public:
    static bool save_csv_in_file(CSVData &data, std::string filename);
};
