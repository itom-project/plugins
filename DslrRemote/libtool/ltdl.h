#ifndef LTDLSUBST_H
#define LTDLSUBST_H

#ifdef DLLEXPORT
#define DECLTYPE extern __declspec(dllexport)
#else
#define DECLTYPE extern __declspec(dllimport)
    //    #define DECLTYPE
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define lt_ptr void *

    typedef	struct lt__handle *lt_dlhandle;	// A loaded module.
    typedef void *	lt_module;

    /* // original struct
    struct lt__handle {
    lt_dlhandle           next;
    const lt_dlvtable *   vtable;		// dlopening interface
    lt_dlinfo				info;		// user visible fields
    int					depcount;	// number of dependencies
    lt_dlhandle *			deplibs;	// dependencies
    lt_module				module;		// system module handle
    void *				system;		// system specific data
    lt_interface_data *	interface_data;	// per caller associated data
    int					flags;		// various boolean stats
    };
    */

    /* Read only information pertaining to a loaded module. */
    typedef	struct lt_dlinfo {
        char *	filename;	// file name
        char *	name;		// module name
        int		ref_count;	// number of times lt_dlopened minus number of times lt_dlclosed.
        unsigned int	is_resident : 1;	// module can't be unloaded.
        unsigned int	is_symglobal : 1;	// module symbols can satisfy subsequently loaded modules.
        /*  unsigned int	is_symlocal:1;	// module symbols are only available locally.
          lt_dlinfo() : filename(NULL), name(NULL), ref_count(0), is_resident(0),
          is_symglobal(0), is_symlocal(0) {};*/
    } lt_dlinfo;

    struct lt__handle {
        lt_dlhandle           next;
        const void *   		vtable;		// dlopening interface
        lt_dlinfo				info;		// user visible fields
        int					depcount;	// number of dependencies
        lt_dlhandle *			deplibs;	// dependencies
        lt_module				module;		// system module handle
        void *				system;		// system specific data
        void *				interface_data;	// per caller associated data
        int					flags;		// various boolean stats
        /*  lt__handle() : next(NULL), vtable(NULL), depcount(0),
              deplibs(NULL), module(NULL), system(NULL), interface_data(NULL), flags(0) {} */
    };

    DECLTYPE int lt_dlinit(void);
    DECLTYPE int lt_dlexit(void);
    DECLTYPE const char * lt_dlerror(void);
    DECLTYPE lt_dlhandle lt_dlopenext(const char *filename);
    DECLTYPE int lt_dlclose(lt_dlhandle handle);
    DECLTYPE void * lt_dlsym(lt_dlhandle handle, const char *name);
    DECLTYPE int lt_dladdsearchdir(const char *search_dir);
    DECLTYPE int lt_dlforeachfile(const char *search_path, int(*func) (const char *filename, void *data), void *data);

#ifdef __cplusplus
}
#endif
#endif // LTDLSUBST_H
