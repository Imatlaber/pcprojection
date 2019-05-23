/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: file io helper
 *
 * Cite:
 *  https://docs.oracle.com/javase/8/docs/api/java/io/File.html
 *  http://stackoverflow.com/questions/146924/how-can-i-tell-if-a-given-path-is-a-directory-or-a-file-c-c
 *  http://stackoverflow.com/questions/5840148/how-can-i-get-a-files-size-in-c
 *  http://stackoverflow.com/questions/612097/how-can-i-get-the-list-of-files-in-a-directory-using-c-or-c
 */

#ifndef _FILE_HELPER_HPP_
#define _FILE_HELPER_HPP_

#include <assert.h>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <functional>
#include <stdexcept>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <dirent.h>
#include <unistd.h>
#include <fstream>
#include <vector>
#include <iostream>
#include <boost/tokenizer.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>


namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace had{

class String {

    // default chars that are considered as whitespace for String::trim*
#define _STRING_TRIM_DELIM_    " \t\r\n"

private:
    String() {
    }

public:
    /**
     * remove trailing whitespace in string source
     */
    static std::string trimRight(std::string const& source, const char* delims = _STRING_TRIM_DELIM_) {
        return std::string(source).erase(source.find_last_not_of(delims) + 1);
    }

public:
    /**
     * remove leading whitespace in string source
     */
    static std::string trimLeft(const std::string & source, const char* delims = _STRING_TRIM_DELIM_) {
        return std::string(source).erase(0, source.find_first_not_of(delims));
    }

public:
    /**
     * remove leading and trailing whitespace in string source
     */
    static std::string trim(const std::string& source, const char* delims = _STRING_TRIM_DELIM_) {
        return trimLeft(trimRight(source, delims), delims);
    }

};

/**
 * class File maintains a string representation of a file
 *
 * the file can be represented either
 *  1) in relative path
 *  2) in absolute path
 *
 * a relative path is mapped to absolute path by prepending
 *  current working directory, i.e. getCwd()
 *
 * functionality to handle
 *  1) environment variables (HOME)
 *  2) shell extensions (~/)
 * are provided as separate static functions
 * and are NOT in-built the File class !
 *
 * the class is modeled after
 *  java.io.File
 *
 * we attempt to be faithful in the aspects
 *  1) function names
 *  2) function definition, including return types
 */
class File {

public:
    /**
     * returns a list of sorted files names given directory, extension and others
     *
     */
    static std::map<int, std::string>  getFileNames(const std::string& dir,
                                                    const std::string& ext,
                                                    int sort_pos_from_end = 0,
                                                    int sort_length = 0, bool stdout = true)
    {
		std::map<int, std::string> files ;
		
		fs::path fpath(dir) ;

        int index = 0 ;
		if(fs::is_directory(fpath))
		{
			for(fs::directory_iterator di(fpath); di != fs::directory_iterator(); ++di)
			{
				if(di->path().extension() == ext)
				{
					std::string path = di->path().string() ;
                    if(sort_pos_from_end > 0 && sort_length > 0)
                    {
					    std::string index_str = path.substr(path.length()-sort_pos_from_end, sort_length) ;
					    index = atoi(index_str.c_str()) ;
                    }
                    else
                    {
                        ++index ;
                    }
					files[index] = path ;
				}
			}
            if (stdout)
			for(auto& ele : files)
			{
				std::cout << ele.second << std::endl ;
			}
		}
		else
		{
			printf("Error: %s is not a valid directory\n", fpath.c_str()) ;
		}
        
        return files ;

    }

    /**
     * returns a list of sorted files names given directory, extension and others
     *
     */
    static std::map<int, std::string>  getdrectories(const std::string& dir,
                                                    int sort_pos_from_end = 0,
                                                    int sort_length = 0, bool stdout = true)
    {
        std::map<int, std::string> files ;

        fs::path fpath(dir) ;

        int index = 0 ;
        if(fs::is_directory(fpath))
        {
            for(fs::directory_iterator di(fpath); di != fs::directory_iterator(); ++di)
            {
                if(fs::is_directory(di->path()))
                {
                    std::string path = di->path().string() ;
                    if(sort_pos_from_end > 0 && sort_length > 0)
                    {
                        std::string index_str = path.substr(path.length()-sort_pos_from_end, sort_length) ;
                        index = atoi(index_str.c_str()) ;
                    }
                    else
                    {
                        ++index ;
                    }
                    files[index] = path ;
                }
            }
            if (stdout)
                for(auto& ele : files)
                {
                    std::cout << ele.second << std::endl ;
                }
        }
        else
        {
            printf("Error: %s is not a valid directory\n", fpath.c_str()) ;
        }

        return files ;

    }


    /**
     * returns current working directory
     *
     * function name as in unistd.h
     */
    static File getCwd() {
        // getcwd: "In GNU, if BUF is NULL, an array is allocated with malloc"
        char* buf = getcwd(nullptr, 0);
        std::string path(buf);
        std::free(buf);
        return File(path);
    }

public:
    /**
     * fromEnv looks up environment variable such as
     *  1) HOME
     *  2) SHELL
     * and returns corresponding file.
     *
     * however, std::runtime_error is thrown if env is undefined.
     */
    static File fromEnv(const std::string& env) {
        // getenv: "Return the value of envariable NAME, or NULL if it doesn't exist."
        char* dir = std::getenv(env.c_str());
        if (dir == nullptr)
            throw std::runtime_error("environment variable [" + env + "] undefined.");
        return File(std::string(dir));
    }

public:
    /**
     * fromEnvAndLocal looks up the first parameter "env" as environment variable
     * and appends the second expression "local" to the path.
     *
     * std::runtime_error is thrown if env is undefined.
     */
    static File fromEnvAndLocal(const std::string& env, const std::string& local) {
        return File(fromEnv(env), local);
    }

public:
    /**
     * fromShell constructs a file from path and
     *  1) replaces leading ~/ with environment variable HOME
     *  2) ... (extension in the future)
     */
    static File fromShell(std::string path) {
        return path.compare(0, 2, "~/") == 0 ? File(fromEnv("HOME"), path.substr(2)) : File(path);
    }

private:
    /**
     * normalize() drops trailing slash '/' from pathname if necessary
     */
    static std::string normalize(const std::string& path) {
        return had::String::trimRight(path, "/");
    }

private:
    // private to allow for change of the representation in the future
    // field path is treated as const
    // however, we don't assign const to preserve default move assignment:
    //  File member;
    //  member = File("readme.txt");
    std::string path; // assumed const, immutable

public:
    /**
     * default constructor results in a non-existing file
     * with an empty path == ""
     */
    File() {
    }

public:
    /**
     * string is a representation of a path name
     *
     * example
     *  1) File("/etc/hosts")
     *  2) File("docs/readme.txt")
     */
    explicit File(const std::string& string) :
        path(normalize(string)) {
    }

public:
    /**
     * constructor to combine
     *  1) relative path + relative path
     *  2) absolute path + relative path
     */
    File(const std::string& prev, const std::string& next) :
        File(File(prev), next) {
    }

public:
    /**
     * constructor to combine
     *  1) relative path + relative path
     *  2) absolute path + relative path
     */
    File(const File& prev, const std::string& next) :
        File(prev, File(next)) {
    }

public:
    /**
     * constructor to combine
     *  1) relative path + relative path
     *  2) absolute path + relative path
     */
    File(const File& prev, const File& next) :
        path(normalize(prev.path + '/' + next.path)) {
        assert(0 < prev.path.length());
        assert(!next.isAbsolute());
    }

public:
    /**
     * returns isAbsolute() ? *this : File(getCwd(), *this);
     */
    File getAbsoluteFile() const {
        return isAbsolute() ? *this : File(getCwd(), *this);
    }

public:
    /**
     * returns true if File represents an existing directory or file
     */
    bool exists() const {
        return isDirectory() || isFile();
    }

public:
    /**
     * quote from java.io.File:
     * """
     * Tests whether the file denoted by this abstract pathname is a directory.
     *
     * returns true if and only if the file denoted by this abstract pathname exists and is a directory; false otherwise
     * """
     */
    // code adapted from answer by Mike F on
    //  http://stackoverflow.com/questions/146924/how-can-i-tell-if-a-given-path-is-a-directory-or-a-file-c-c
    bool isDirectory() const {
        struct stat diag;
        if (stat(path.c_str(), &diag) == 0) {
            if (diag.st_mode & S_IFDIR) {
                // it's a directory
                return true;
            } else if (diag.st_mode & S_IFREG) {
                // it's a file
            } else {
                // something else
            }
        } else {
            // error
        }
        return false;
    }

public:
    /**
     * quote from java.io.File:
     * """
     * Tests whether the file denoted by this abstract pathname is a normal file.
     * A file is normal if it is not a directory and, in addition, satisfies other system-dependent criteria.
     *
     * returns true if and only if the file denoted by this abstract pathname exists and is a normal file; false otherwise
     * """
     */
    // code adapted from answer by Mike F on
    //  http://stackoverflow.com/questions/146924/how-can-i-tell-if-a-given-path-is-a-directory-or-a-file-c-c
    bool isFile() const {
        struct stat diag;
        if (stat(path.c_str(), &diag) == 0) {
            if (diag.st_mode & S_IFDIR) {
                // it's a directory
            } else if (diag.st_mode & S_IFREG) {
                // it's a file
                return true;
            } else {
                // something else
            }
        } else {
            // error
        }
        return false;
    }

public:
    /**
     * returns number of bytes of file, or -1 if counting bytes is not a meaningful operation
     */
    // code adapted from Spyros on
    //  http://stackoverflow.com/questions/5840148/how-can-i-get-a-files-size-in-c
    long length() const {
        return isFile() ? long(std::ifstream(path.c_str(), std::ifstream::ate | std::ifstream::binary).tellg()) : -1;
    }

public:
    /**
     * Delete the file or directory given by the path. Return true iff successful.
     *
     * function has the same effect as
     *  java.io.File::delete()
     */
    bool remove() const {
        return 0 == std::remove(path.c_str());
    }

public:
    /**
     * Create the directory given by the path. Return true iff successful.
     *
     * The call to mkdir() herein uses the 0775 mask, which corresponds to the default as
     *
     *  java.io.File::mkdir(), and
     *  mkdir (the shell command as normal ubuntu user)
     *
     * (Other access permissions are currently not supported.)
     */
    bool makeDirectory() const {
        return 0 == mkdir(path.c_str(), 0775);
    }

public:
    /**
     * returns true if File defines an absolute location in the file system
     */
    bool isAbsolute() const {
        return 0 < path.length() && path.at(0) == '/';
    }

public:
    /**
     * returns collection of files in the directory
     *  if this->isDirectory() == true
     *
     * in order to match the behaviour of the respective
     * function java.io.File::listFiles()
     * the files/links:
     *  '.', and
     *  '..'
     * are not included in the list.
     *
     * returns an empty vector if *this file is not a directory
     *  i.e. this->isDirectory() == false
     *
     * example use:
     *  for (File file : File::fromShell("~/documents").listFiles()) {
     *    ... handle file from HOME/documents directory ...
     *  }
     */
    // code adapted from Peter Parker
    //  http://stackoverflow.com/questions/612097/how-can-i-get-the-list-of-files-in-a-directory-using-c-or-c
    std::vector<File> listFiles() {
        std::vector<File> vector;
        DIR* dir;
        struct dirent* ent;
        if ((dir = opendir(path.c_str())) != nullptr) {
            while ((ent = readdir(dir)) != nullptr)
                if (strcmp(ent->d_name, ".") == 0) {
                    // omit .
                } else if (strcmp(ent->d_name, "..") == 0) {
                    // omit ..
                } else
                    vector.push_back(File(path, std::string(ent->d_name)));
            closedir(dir);
        } else {
            // could not open directory
            // perror("");
        }
        return vector;
    }

public:
    /**
     * returns string representation of path
     */
    std::string toString() const {
        return path;
    }

public:
    /**
     * returns c_str() invoked on string representation of path (for convenience)
     */
    const char* c_str() const {
        return path.c_str();
    }

public:
    /**
     * equality operator helps to identify identical files
     *
     * for instance
     *  File("/home/") == File("/home")
     */
    bool operator==(const File& rhs) const {
        return path == rhs.path;
    }

public:
    /**
     * operator< is implemented to support use in ordered sets such as
     *  set<File>
     */
    bool operator<(const File& rhs) const {
        return path < rhs.path;
    }

};

}

namespace std {

/**
 * hash is implemented to support use in unordered sets such as
 *  unordered_set<File>
 */
template<>
struct hash<had::File> {
    size_t operator()(const had::File& file) const {
        return std::hash<std::string>()(file.toString());
    }
};

}


#endif
