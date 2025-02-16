#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

int main() {
    // Read the CSV file
    std::ifstream file("trajectories/traj1.csv");
    if (!file) {
        std::cout << "Failed to open traj1.csv" << std::endl;
        return 1;
    }
    
    // Read the header row
    std::string line;
    if (!std::getline(file, line)) {
        std::cout << "Failed to read the header row" << std::endl;
        return 1;
    }
    
    // Split the header row into column names
    std::vector<std::string> columnNames;
    std::string columnName;
    std::istringstream iss(line);
    while (std::getline(iss, columnName, ',')) {
        columnNames.push_back(columnName);
    }
    
    // Determine the number of groups
    int numGroups = columnNames.size() / 6;
    
    // Create data structures to store the groups
    std::vector<std::vector<double>> posGroup, velGroup, accGroup;
    
    // Read and store the data for each group
    std::string value;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        for (int groupIndex = 0; groupIndex < numGroups; ++groupIndex) {
            std::vector<double> groupData;
            for (int columnIndex = groupIndex * 6; columnIndex < (groupIndex + 1) * 6; ++columnIndex) {
                std::getline(iss, value, ',');
                groupData.push_back(std::stod(value));
            }
            if (groupIndex == 0) {
                posGroup.push_back(groupData);
            } else if (groupIndex == 1) {
                velGroup.push_back(groupData);
            } else if (groupIndex == 2) {
                accGroup.push_back(groupData);
            }
        }
    }
    
    // Print the stored data for each group
    std::cout << "pos:" << std::endl;
    for (const auto& group : posGroup) {
        for (const auto& value : group) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }
    
    // std::cout << "vel:" << std::endl;
    // for (const auto& group : velGroup) {
    //     for (const auto& value : group) {
    //         std::cout << value << " ";
    //     }
    //     std::cout << std::endl;
    // }
    
    // std::cout << "acc:" << std::endl;
    // for (const auto& group : accGroup) {
    //     for (const auto& value : group) {
    //         std::cout << value << " ";
    //     }
    //     std::cout << std::endl;
    // }
    
    // Close the input file
    file.close();

    return 0;
}
