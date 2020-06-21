#pragma once

#include <string>
#include <vector>

namespace rpg_common
{
namespace fs
{

bool fileExists(const std::string& path);
bool pathExists(const std::string& path);

void splitPathAndFilename(
    const std::string& str, std::string* path, std::string* filename);

// Returns full paths.
void getFilesAndSubfolders(const std::string& path,
                           std::vector<std::string>* files,
                           std::vector<std::string>* folders);

}  // namespace fs
}  // namespace rpg_common

namespace rpg = rpg_common;
