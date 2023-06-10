#include <Windows.h>
#include <map>
#include <assert.h>
#include <string>
#include "ltdl.h"

const char EOS_CHAR = '\0';
const char PATHSEP_CHAR = ';';
const char DIRSEP_CHAR = '\\';
const char ALTDIRSEP_CHAR = '/';
static char errorTxtBuf[2048];
static std::map<std::string, lt__handle> loadedLibs;

//----------------------------------------------------------------------------------------------------------------------------------
static int canonicalize_path(const char *path, char **pcanonical)
{
    char *canonical = 0;
    assert(path && *path);
    assert(pcanonical);

    canonical = (char*)calloc(sizeof(char), 1 + strlen(path));
    if (!canonical)
        return 1;

    size_t dest = 0;
    size_t src;
    for (src = 0; path[src] != EOS_CHAR; ++src)
    {
        // Path separators are not copied to the beginning or end of
        // the destination, or if another separator would follow
        // immediately.
        if (path[src] == PATHSEP_CHAR)
        {
            if ((dest == 0)
                || (path[1 + src] == PATHSEP_CHAR)
                || (path[1 + src] == EOS_CHAR))
                continue;
        }

        // Anything other than a directory separator is copied verbatim.
        if ((path[src] != PATHSEP_CHAR) && (path[src] != DIRSEP_CHAR) && (path[src] != ALTDIRSEP_CHAR))
        {
            canonical[dest++] = path[src];
        }
        // Directory separators are converted and copied only if they are
        // not at the end of a path -- i.e. before a path separator or
        // NULL terminator.
        else if ((path[1 + src] != PATHSEP_CHAR)
            && (path[1 + src] != EOS_CHAR)
            && (path[1 + src] != DIRSEP_CHAR)
            && (path[1 + src] != ALTDIRSEP_CHAR))
        {
            canonical[dest++] = '/';
        }
    }

    // Add an end-of-string marker at the end.
    canonical[dest] = EOS_CHAR;

    // Assign new value.
    *pcanonical = canonical;

    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
const char * lt_dlerror(void)
{
    DWORD errorMessageID = GetLastError();
    if(errorMessageID == 0)
		return ""; //No error message has been recorded

    LPSTR messageBuffer = NULL;
    size_t size = FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
                                 NULL, errorMessageID, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&messageBuffer, 0, NULL);

	 strncpy(errorTxtBuf, messageBuffer, 2048);
	 errorTxtBuf[2047] = 0;

    //Free the buffer.
    LocalFree(messageBuffer);

    return errorTxtBuf;
}

//----------------------------------------------------------------------------------------------------------------------------------
int lt_dlinit(void)
{
    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
int lt_dlexit(void)
{
    for (std::map<std::string, lt__handle>::iterator it = loadedLibs.begin();
        it != loadedLibs.end(); ++it)
    {
        FreeLibrary((HMODULE)it->second.vtable);
    }
    loadedLibs.clear();
    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
lt_dlhandle lt_dlopenext(const char *filename)
{
    char *canonical = NULL;
    char * dir = NULL;
    lt_dlhandle ret = NULL;
    if (canonicalize_path(filename, &canonical) != 0)
    {
        goto cleanup;
    }

    // If the canonical module name is a path (relative or absolute)
    // then split it into a directory part and a name part.
    char * base_name = strrchr(canonical, '/');
    if (base_name)
    {
        size_t dirlen = (1 + base_name) - canonical;

        dir = (char*)calloc(sizeof(char), 1 + dirlen);
        if (!dir)
        {
            goto cleanup;
        }

        strncpy(dir, canonical, dirlen);
        dir[dirlen] = EOS_CHAR;

        ++base_name;
    }
    else
    {
//        MEMREASSIGN(base_name, canonical);
        base_name = canonical;
    }

    assert(base_name && *base_name);

    const char *ext = strrchr(base_name, '.');
    if (!ext)
    {
        ext = base_name + strlen(base_name);
    }
    else
        base_name[ext - base_name] = EOS_CHAR;

    if (loadedLibs.find(base_name) != loadedLibs.end())
    {
        ret = &loadedLibs[base_name];
        loadedLibs[base_name].depcount++;
        goto cleanup;
    }
    else
    {
        HMODULE lib = LoadLibraryA(filename);
        if (lib)
        {
            lt__handle libHandle;
            libHandle.depcount = 0;
            libHandle.flags = 0;
            libHandle.deplibs = NULL;
            libHandle.interface_data = NULL;
            libHandle.module = NULL;
            libHandle.next = NULL;
            libHandle.system = NULL;
            libHandle.info.name = NULL;
            libHandle.info.is_resident = 0;
            libHandle.info.is_symglobal = 0;
            libHandle.info.ref_count = 0;
            libHandle.info.filename = (char*)calloc(sizeof(char), strlen(filename) + 1);
            strncpy(libHandle.info.filename, filename, strlen(filename));
            libHandle.vtable = lib;
            loadedLibs[std::string(base_name)] = libHandle;
            ret = &loadedLibs[base_name];
            goto cleanup;
        }
    }

cleanup:
    if (canonical)
        free(canonical);
    if (dir)
        free(dir);
    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
int lt_dlclose(lt_dlhandle libhandle)
{
    for (std::map<std::string, lt__handle>::iterator it = loadedLibs.begin();
        it != loadedLibs.end(); ++it)
    {
        if (&it->second == libhandle)
        {
            it->second.depcount--;
            if (it->second.depcount <= 0)
            {
                FreeLibrary((HMODULE)it->second.vtable);
                loadedLibs.erase(it);
            }
            break;
        }
    }
    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
void * lt_dlsym(lt_dlhandle libhandle, const char *name)
{
    void *funcAddr = GetProcAddress((HMODULE)libhandle->vtable, name);
//    if (funcAddr)
//        libhandle->depcount++;
    return funcAddr;
}

//----------------------------------------------------------------------------------------------------------------------------------
int lt_dladdsearchdir(const char *search_dir)
{
    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
int lt_dlforeachfile(const char *search_path, int(*func) (const char *filename, void *data), void *data)
{
    int retval = 0;
    char *canonical = NULL;
    char *searchname = NULL;
    char *tmpFile = NULL;
    int maxsize = 255;
    WIN32_FIND_DATAA FindFileData;
    assert(search_path);

    if (canonicalize_path(search_path, &canonical) != 0)
    {
        goto cleanup;
    }

    // If the canonical module name is a path (relative or absolute)
    // then split it into a directory part and a name part.
    char * base_name = strrchr(canonical, '/');

    // already ends with directory separator, so we just add our search pattern
    if (!base_name)
    {
        searchname = (char*)calloc(sizeof(char), strlen(canonical) + 6);
        strncpy(searchname, canonical, strlen(canonical));
        searchname = strcat(searchname, "*.dll");
    }
    else
    {
        base_name = strrchr(canonical, '.');
        // not a directory nor something that could be a file pattern so we glue
        // what is for us a meaningful ending ...
        if (!base_name)
        {
            searchname = (char*)calloc(sizeof(char), strlen(canonical) + 7);
            strncpy(searchname, canonical, strlen(canonical));
            searchname = strcat(searchname, "/*.dll");
        }
        else
        {
            searchname = (char*)calloc(sizeof(char), strlen(canonical) + 1);
            strncpy(searchname, canonical, strlen(canonical));
        }
    }
    HANDLE dirh = FindFirstFileA(searchname, &FindFileData);
    if (dirh != INVALID_HANDLE_VALUE)
    {
        int canlen = strlen(canonical);
        tmpFile = (char*)calloc(sizeof(char), canlen + maxsize);
        if (canonical[canlen] != '/')
        {
            canonical = (char*)realloc(canonical, canlen + 2);
            canonical = strcat(canonical, "/");
            canlen++;
        }
        tmpFile = strncpy(tmpFile, canonical, canlen);
        if (strlen(FindFileData.cFileName) >= 250)
        {
            maxsize = strlen(FindFileData.cFileName + 10);
            tmpFile = (char*)realloc(tmpFile, maxsize);
        }
        if (retval += func(strcat(tmpFile, FindFileData.cFileName), data))
            goto cleanup;
        while (FindNextFileA(dirh, &FindFileData))
        {
            memset(tmpFile, 0, maxsize);
            if (strlen(FindFileData.cFileName) >= 250)
            {
                maxsize = strlen(FindFileData.cFileName + 10);
                tmpFile = (char*)realloc(tmpFile, maxsize);
            }
            tmpFile = strncpy(tmpFile, canonical, canlen);
            if (retval += func(strcat(tmpFile, FindFileData.cFileName), data))
                goto cleanup;
        }
    }

cleanup:
    if (canonical)
        free(canonical);
    if (searchname)
        free(searchname);
    if (tmpFile)
        free(tmpFile);
    if (dirh != INVALID_HANDLE_VALUE)
        FindClose(dirh);
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
