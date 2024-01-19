#include <iostream>
#include <fstream>
#include <vector>
#include <string>

using namespace std;

vector<vector<double>> read_csv(string filename) {
    vector<vector<double>> data;
    ifstream file(filename);
    if (!file.is_open()){
        cout << "Error opening fle!" << endl;
        exit(1);
    }
    string line;
    while (getline(file, line)) {
        vector<double> row;
        size_t pos = 0;
        string token;
        while ((pos = line.find(',')) != string::npos) {
            token = line.substr(0, pos);
            row.push_back(stod(token));
            line.erase(0, pos + 1);
        }
        row.push_back(stod(line));
        data.push_back(row);
    }
    return data;
}

// int main() {
//     vector<vector<double>> data = read_csv("filename.csv");
//     for (auto row : data) {
//         for (auto cell : row) {
//             cout << cell << " ";
//         }
//         cout << endl;
//     }
//     return 0;
// }