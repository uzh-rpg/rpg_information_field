//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "rpg_common/fs.h"

#include <dirent.h>
#include <errno.h>
#include <sys/stat.h>

#include <glog/logging.h>

namespace rpg_common
{
namespace fs
{

bool fileExists(const std::string& path)
{
  struct stat st;
  return stat(path.c_str(), &st) == 0 && (st.st_mode & S_IFREG);
}

bool pathExists(const std::string& path)
{
  struct stat st;
  return stat(path.c_str(), &st) == 0 && (st.st_mode & S_IFDIR);
}

void splitPathAndFilename(
    const std::string& str, std::string* path, std::string* filename)
{
  CHECK_NOTNULL(path)->clear();
  CHECK_NOTNULL(filename)->clear();
  const size_t right_delim = str.find_last_of("/");
  if (right_delim != std::string::npos)
  {
    *path = str.substr(0, right_delim);
  }
  *filename = str.substr(right_delim + 1);
}

// Returns full paths. No recursion.
void getFilesAndSubfolders(const std::string& path,
                           std::vector<std::string>* files,
                           std::vector<std::string>* folders)
{
  CHECK_NOTNULL(files)->clear();
  CHECK_NOTNULL(folders)->clear();
  CHECK(pathExists(path));

  DIR* directory_stream = CHECK_NOTNULL(opendir(path.c_str()));
  struct dirent* directory_entry;
  while ((directory_entry = readdir(directory_stream)) != NULL)
  {
    const std::string filename(directory_entry->d_name);
    if ((filename == ".") || (filename == "..")) {
      continue;
    }
    const std::string abs_path = path + "/" + filename;

    if (fileExists(abs_path))
    {
      files->emplace_back(abs_path);
    }
    else
    {
      CHECK(pathExists(abs_path));
      folders->emplace_back(abs_path);
    }
  }

  closedir(directory_stream);
}

}  // namespace fs
}  // namespace rpg_common
