# odin-janet

barebones odin bindings for janet

I HAVE NOT EVEN FULLY TESTED THIS.

TODOs:
    - Rewrite the macros written with #force_inline to use mem operations instead of uintptr
    - Test all bindings
    - Write some wrappers to use slices instead of multi-pointers+length
    - Package janet.c and a makefile to let the user compile with their own flags
    - Add other implementations instead of only using NANBOX64 (or use the given functions instead?, the performance difference should be minimal)
    - I think a couple of interfaces are missing
