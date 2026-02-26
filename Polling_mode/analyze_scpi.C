/**
 * @file analyze_scpi.C
 * @version 1.0.0
 * @date 2026-02-26
 * @author Leonardo Lisa
 * @brief SCPI Multichannel Analyzer ROOT Macro for dynamic data parsing and plotting.
 * @requirements CERN ROOT Framework.
 * @execution root -l 'analyze_scpi.C("filename.dat")'
 * * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <TString.h>
#include <TCanvas.h>
#include <TH1D.h>
#include <TMath.h>
#include <TStyle.h>
#include <TPad.h>

#define ANSI_CYAN    "\x1b[36m"
#define ANSI_RED     "\x1b[31m"
#define ANSI_RESET   "\x1b[0m"

void analyze_scpi(const char* filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << ANSI_RED << "[Error] Cannot open file: " << filename << ANSI_RESET << "\n";
        return;
    }

    std::string header_line;
    // Extract the first line and verify it is a valid SCPI header
    if (!std::getline(file, header_line) || header_line.empty() || header_line[0] != '#') {
        std::cerr << ANSI_RED << "[Error] Invalid or missing header in file." << ANSI_RESET << "\n";
        return;
    }

    std::istringstream iss(header_line);
    std::string token;
    std::vector<std::string> measurements;

    iss >> token; // Discard the '#' prefix
    while (iss >> token) {
        if (token != "timestamp") {
            measurements.push_back(token);
        }
    }

    if (measurements.empty()) {
        std::cerr << ANSI_RED << "[Error] No valid measurements found in header." << ANSI_RESET << "\n";
        return;
    }

    // UI Presentation
    std::cout << ANSI_CYAN << "[Data Parser]" << ANSI_RESET << " Found the following measurements:\n";
    for (size_t i = 0; i < measurements.size(); ++i) {
        std::cout << "  [" << i << "] " << measurements[i] << "\n";
    }

    std::cout << "\nEnter the index or name of the measurement to plot: ";
    std::string user_input;
    std::cin >> user_input;

    int selected_idx = -1;
    std::string selected_name;

    // Parse user input (supports both integer index and explicit string name)
    try {
        size_t idx = std::stoul(user_input);
        if (idx < measurements.size()) {
            selected_idx = idx;
            selected_name = measurements[idx];
        }
    } catch (...) {
        auto it = std::find(measurements.begin(), measurements.end(), user_input);
        if (it != measurements.end()) {
            selected_idx = std::distance(measurements.begin(), it);
            selected_name = *it;
        }
    }

    if (selected_idx == -1) {
        std::cerr << ANSI_RED << "[Error] Invalid selection." << ANSI_RESET << "\n";
        return;
    }

    std::vector<double> data;
    std::string line;
    uint32_t dropped_events = 0;

    // Data Extraction Loop
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream line_stream(line);
        std::string val_str;
        
        // Advance stream to the target column
        for (int i = 0; i <= selected_idx; ++i) {
            line_stream >> val_str;
        }

        // Filter SCPI read failures
        if (val_str == "?" || val_str == "NAN" || val_str == "INF") {
            dropped_events++;
            continue;
        }

        try {
            data.push_back(std::stod(val_str));
        } catch (...) {
            dropped_events++; // Handle malformed numerical strings
        }
    }

    if (data.empty()) {
        std::cerr << ANSI_RED << "[Error] No valid numeric data found for " << selected_name << ANSI_RESET << "\n";
        return;
    }

    // Statistical Computation
    double mean = TMath::Mean(data.begin(), data.end());
    double stddev = TMath::RMS(data.begin(), data.end());

    // Boundary Definition: Centered on mean, width constrained to exactly +/- 1 Sigma
    double n_sigma = 1.0; 
    double x_min = mean - (n_sigma * stddev);
    double x_max = mean + (n_sigma * stddev);

    // Fallback for zero variance data to prevent ROOT histogram axis corruption
    if (stddev == 0) {
        x_min = mean - 1.0;
        x_max = mean + 1.0;
    }

    // Histogram Instantiation
    gStyle->SetOptStat(111111); 
    
    TCanvas* c1 = new TCanvas("c1", Form("%s Histogram", selected_name.c_str()), 800, 600);
    c1->SetGrid();

    // Binning optimization: Fixed 100 bins for the 1-sigma domain
    TH1D* hist = new TH1D("h_meas", Form("%s Nominal Distribution (1#sigma);%s;Counts", selected_name.c_str(), selected_name.c_str()), 100, x_min, x_max);
    hist->SetLineColor(kBlue + 1);
    hist->SetFillColor(kBlue - 9);
    hist->SetLineWidth(2);

    for (double val : data) {
        hist->Fill(val);
    }

    hist->Draw();
    c1->Update();

    std::cout << ANSI_CYAN << "\n[Analysis Complete]\n" << ANSI_RESET;
    std::cout << "Measurement:\t\t" << selected_name << "\n";
    std::cout << "Valid samples:\t\t" << data.size() << "\n";
    std::cout << "Filtered samples:\t" << dropped_events << "\n";
    std::cout << "Distribution Mean:\t" << mean << "\n";
    std::cout << "Distribution StdDev:\t" << stddev << "\n";
}