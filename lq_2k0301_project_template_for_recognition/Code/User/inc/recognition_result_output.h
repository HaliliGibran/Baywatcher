#pragma once

#include <string>

// Must run before board initialization so startup and library logs are also silenced.
void ConfigureRecognitionResultOutput();

// Writes through the saved terminal handle when exclusive result output is enabled.
void PrintRecognitionResultLine(const std::string& line);
