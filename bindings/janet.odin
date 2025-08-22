package janet

STATIC :: true

NANBOX_64 :: true //HACK: ignoring anything but my use case
ASSEMBLER :: true
PRF :: true
PEG :: true
INT_TYPES :: true
JANET_EV :: true


when STATIC {
    foreign import janet "include/janet.a"
}

when ODIN_OS == .Windows {
    Handle :: rawptr
    HANDLE_NONE :: nil
} else {
    Handle :: i32
    HANDLE_NONE :: -1
}


AtomicInt :: i32


import "core:c/libc"
import "core:c"
import "core:os"
import "base:intrinsics"
import "base:builtin"



VERSION_MAJOR       :: 1
VERSION_MINOR       :: 38
VERSION_PATCH       :: 0
VERSION_EXTRA       :: ""
VERSION             :: "1.38.0"
CURRENT_CONFIG_BITS :: 0 | 0x1

BuildConfig :: struct {
    major: uint,
    minor: uint,
    patch: uint,
    bits: uint,
}

config_current :: BuildConfig{
    VERSION_MAJOR,
    VERSION_MINOR,
    VERSION_PATCH,
    CURRENT_CONFIG_BITS,
}

/* Fiber signals */
Signal :: enum {
    OK,
    ERROR,
    DEBUG,
    YIELD,
    USER0,
    USER1,
    USER2,
    USER3,
    USER4,
    USER5,
    USER6,
    USER7,
    USER8,
    USER9,
    INTERRUPT = USER8,
    EVENT     = USER9,
}

/* Fiber statuses - mostly corresponds to signals. */
FiberStatus :: enum {
    DEAD,
    ERROR,
    DEBUG,
    PENDING,
    USER0,
    USER1,
    USER2,
    USER3,
    USER4,
    USER5,
    USER6,
    USER7,
    USER8,
    USER9,
    NEW,
    ALIVE
}

/* For encapsulating all thread-local Janet state (except natives) */
/*opaque pointer*/
VM :: struct {}


/* GC Object type pun. The lower 16 bits of flags are reserved for the garbage collector,
 * but the upper 16 can be used per type for custom flags. The current collector is a linked
 * list of blocks, which is naive but works. */
GCObject :: struct {
    flags: i32,
    data: struct #raw_union {
        next: ^GCObject,
        refcount: AtomicInt, /* For threaded abstract types */
    }
}

/* All of the primary Janet GCed types */

/* A function */
Function :: struct {
    gc: GCObject,
    def: ^FuncDef,
    envs: [^]^FuncEnv,
}

/* A dynamic array type. */
Array :: struct {
    gc: GCObject,
    count: i32,
    capacity: i32,
    data: ^Janet,
}

/* A byte buffer type. Used as a mutable string or string builder. */
Buffer :: struct {
    gc: GCObject,
    count: i32,
    capacity: i32,
    data: ^u8,
}

/* A mutable associative data type. Backed by a hashtable. */
Table :: struct {
    gc: GCObject,
    count: i32,
    capacity: i32,
    deleted: i32,
    data: ^KV,
    proto: ^Table,
}

/* A lightweight green thread in janet. Does not correspond to
 * operating system threads. */
Fiber :: struct {
    gc: GCObject, /* GC Object stuff */
    flags: i32, /* More flags */
    frame: i32, /* Index of the stack frame */
    stackstart: i32, /* Beginning of next args */
    stacktop: i32, /* Top of stack. Where values are pushed and popped from. */
    capacity: i32, /* How big is the stack memory */
    maxstack: i32, /* Arbitrary defined limit for stack overflow */
    env: ^Table, /* Dynamic bindings table (usually current environment). */
    data: ^Janet, /* Dynamically resized stack memory */
    child: ^Fiber, /* Keep linked list of fibers for restarting pending fibers */
    last_value: Janet, /* Last returned value from a fiber */

    //HACK: Janet is compiled as a native target only,
    //you must recompile it and adjust this for web use
    /* These fields are only relevant for fibers that are used as "root fibers" -
    * that is, fibers that are scheduled on the event loop and behave much like threads
    * in a multi-tasking system. It would be possible to move these fields to a new
    * type, say "JanetTask", that as separate from fibers to save a bit of space. */
    using _: Janet_Ev
}


/* Prefixed Janet types */
TupleHead :: struct {
    gc: GCObject,
    length: i32,
    hash: i32,
    sm_line: i32,
    sm_column: i32,
    data: [^]Janet
}

StructHead :: struct {
    gc: GCObject,
    length: i32,
    hash: i32,
    capacity: i32,
    proto: ^KV,
    data: [^]KV,
}

StringHead :: struct {
    gc: GCObject,
    length: i32,
    hash: i32,
    data: [^]u8
}
AbstractHead :: struct {
    gc: GCObject,
    type: ^AbstractType,
    size: uint,
    data: [^]i64, /* Use long long to ensure most general alignment */
}



/* Other structs */

/* A function definition. Contains information needed to instantiate closures. */
FuncDef :: struct {
    gc: GCObject,
    environments: [^]i32, /* Which environments to capture from parent. */
    constants: [^]Janet,
    defs: [^]^FuncDef,
    bytecode: [^]u32,
    closure_bitset: ^u32, /* Bit set indicating which slots can be referenced by closures. */

    /* Various debug information */
    sourcemap: ^SourceMapping,
    source: String,
    name: String,
    symbolmap: [^]SymbolMap,

    flags: i32,
    slotcount: i32, /* The amount of stack space required for the function */
    arity: i32, /* Not including varargs */
    min_arity: i32, /* Including varargs */
    max_arity: i32, /* Including varargs */
    constants_length: i32,
    bytecode_length: i32,
    environments_length: i32,
    defs_length: i32,
    symbolmap_length: i32,
}

/* A function environment */
FuncEnv :: struct {
    gc: GCObject,
    as: struct #raw_union {
        fiber: ^Fiber,
        values: ^Janet,
    },
    length: i32, /* Size of environment */
    offset: i32, /* Stack offset when values still on stack. If offset is <= 0, then
        environment is no longer on the stack. */
}

/* A key value pair in a struct or table */
KV :: struct { key, value: Janet }

/* A stack frame on the fiber. Is stored along with the stack values. */
StackFrame :: struct  {
    func: ^Function,
    pc: ^u32,
    env: ^FuncEnv,
    prevframe: i32,
    flags: i32,
}

/* Defines an abstract type */
AbstractType :: struct {
    name: builtin.cstring,
    gc:        proc "c" (data: rawptr, len: uint) -> i32,
    gcmark:    proc "c" (data: rawptr, len: uint) -> i32,
    get:       proc "c" (data: rawptr, key: Janet , out: ^Janet ) -> i32,
    put:       proc "c" (data: rawptr, key, value: Janet),
    marshal:   proc "c" (p: rawptr, ctx: ^MarshalContext),
    unmarshal: proc "c" (ctx: ^MarshalContext),
    tostring:  proc "c" (p: rawptr, buffer: ^Buffer),
    compare:   proc "c" (lhs, rhs: rawptr) -> i32,
    hash:      proc "c" (p: rawptr, len: uint) -> i32,
    next:      proc "c" (p: rawptr, key: Janet) -> Janet,
    call:      proc "c" (p: rawptr, argc: i32, argv: [^]Janet ) -> Janet,
    length:    proc "c" (p: rawptr, len: uint) -> uint,
    bytes:     proc "c" (p: rawptr, len: uint) -> ByteView,
}

Reg :: struct {
    name: builtin.cstring,
    cfun: CFunction,
    documentation: builtin.cstring,
}

RegExt :: struct {
    name: builtin.cstring,
    cfun: CFunction,
    documentation: builtin.cstring,
    source_file: builtin.cstring,
    source_line: i32,
}

Method :: struct {
    name: builtin.cstring,
    cfun: CFunction,
}

/* Source mapping structure for a bytecode instruction */
SourceMapping :: struct {
    line: i32,
    column: i32,
}

/* Symbol to slot mapping & lifetime structure. */
SymbolMap :: struct {
    birth_pc: u32,
    death_pc: u32,
    slot_index: u32,
    symbol: ^u8,
}

View :: struct{
    items: [^]Janet,
    len: i32,
}

ByteView :: struct{
    bytes: [^]u8,
    len: i32,
}

DictView :: struct{
    kvs: [^]KV,
    len: i32,
    cap: i32,
}

Range :: struct{
    start: i32,
    end: i32,
}

RNG :: struct{
    a, b, c, d: u32,
    counter: u32,
}

/* Buffer functions */
BUFFER_FLAG_NO_REALLOC :: 0x10000

Type :: enum u32 {
    NUMBER,
    NIL,
    BOOLEAN,
    FIBER,
    STRING,
    SYMBOL,
    KEYWORD,
    ARRAY,
    TUPLE,
    TABLE,
    STRUCT,
    BUFFER,
    FUNCTION,
    CFUNCTION,
    ABSTRACT,
    POINTER
}

CFunction :: #type proc "c" (argc: i32, argv: [^]Janet) -> Janet

/* String and other aliased pointer types */
String   :: builtin.cstring
Symbol   :: builtin.cstring
Keyword  :: builtin.cstring
Tuple    :: Janet
Struct   :: ^KV
Abstract :: rawptr

COUNT_TYPES  :: len(Type) + 1

/* Type flags */
Type_Flags :: bit_set[Type]

T_NIL :: Type_Flags{.NIL}
T_BOOLEAN :: Type_Flags{.BOOLEAN}
T_FIBER :: Type_Flags{.FIBER}
T_NUMBER :: Type_Flags{.NUMBER}
T_STRING :: Type_Flags{.STRING}
T_SYMBOL :: Type_Flags{.SYMBOL}
T_KEYWORD :: Type_Flags{.KEYWORD}
T_ARRAY :: Type_Flags{.ARRAY}
T_TUPLE :: Type_Flags{.TUPLE}
T_TABLE :: Type_Flags{.TABLE}
T_STRUCT :: Type_Flags{.STRUCT}
T_BUFFER :: Type_Flags{.BUFFER}
T_FUNCTION :: Type_Flags{.FUNCTION}
T_CFUNCTION :: Type_Flags{.CFUNCTION}
T_ABSTRACT :: Type_Flags{.ABSTRACT}
T_POINTER :: Type_Flags{.POINTER}

T_BYTES      :: T_STRING | T_SYMBOL | T_BUFFER | T_KEYWORD
T_INDEXED    :: T_ARRAY | T_TUPLE
T_DICTIONARY :: T_TABLE | T_STRUCT
T_LENGTHABLE :: T_BYTES | T_INDEXED | T_DICTIONARY
T_CALLABLE   :: T_FUNCTION | T_CFUNCTION | T_LENGTHABLE | T_ABSTRACT

checktypes :: #force_inline proc(x: $T, tps: Type_Flags) { return (1 << type(x)) & (tps)}
when NANBOX_64 {

    Janet :: struct #raw_union {
        u64: u64,
        i64: i64,
        number: f64,
        pointer: rawptr,
    }

    NANBOX_TAGBITS     : u64 : 0xFFFF800000000000
    NANBOX_PAYLOADBITS : u64 : 0x00007FFFFFFFFFFF

    nanbox_lowtag :: #force_inline proc(t: Type) -> u64 { return u64(t) | 0x1FFF0 }
    nanbox_tag :: #force_inline proc(t: Type) -> u64 { return nanbox_lowtag(t) << 47 }

    type :: #force_inline proc(x: Janet) -> (result: Type) {
        if libc.isnan(x.number) {
            result = Type((x.u64 >> 47) & 0xF)
        }
        else do result = .NUMBER
        return
    }

    nanbox_checkauxtype :: #force_inline proc(x: Janet, t: Type) -> bool {
        return (x.u64 & NANBOX_TAGBITS) == nanbox_tag(t)
    }

    nanbox_isnumber :: #force_inline proc(x: Janet) -> bool {
        return !libc.isnan(x.number) || ((x.u64 >> 47) & 0xF == u64(Type.NUMBER))
    }

    checktype :: #force_inline proc(x: Janet, t: Type) -> bool {
        return nanbox_isnumber(x) if t == .NUMBER else nanbox_checkauxtype(x, t)
    }

    truthy :: #force_inline proc(x: Janet) -> bool {
        return !checktype(x, .NIL) &&
            (!checktype(x, .BOOLEAN) || bool(x.u64 & 0x1))
    }

    nanbox_from_payload :: #force_inline proc(t: Type, p: u64) -> Janet {
        return nanbox_from_bits(nanbox_tag(t) | p)
    }

    nanbox_wrap_ :: #force_inline proc(p: rawptr, t: Type) -> Janet {
        return nanbox_from_pointer(p, nanbox_tag(t))
    }

    nanbox_wrap_c :: #force_inline proc(p: rawptr, t: Type) -> Janet {
        return nanbox_from_cpointer(p, nanbox_tag(t))
    }


    /* Wrap the simple types */
    wrap_nil :: #force_inline proc() -> Janet { return nanbox_from_payload(.NIL, 1) }
    wrap_true :: #force_inline proc() -> Janet { return nanbox_from_payload(.BOOLEAN, 1) }
    wrap_false :: #force_inline proc() -> Janet { return nanbox_from_payload(.BOOLEAN, 0) }
    wrap_boolean :: #force_inline proc(b: bool) -> Janet { return nanbox_from_payload(.BOOLEAN, u64(b)) }
    wrap_number :: #force_inline proc(r: f64) -> Janet { return nanbox_from_double(r) }

    /* Unwrap the simple types */
    unwrap_boolean :: #force_inline proc(x: Janet) -> bool {return bool(x.u64 & 0x1)}
    unwrap_number :: #force_inline proc(x: Janet) -> u64 {return transmute(u64) x.number }

    /* Wrap the pointer types */
    wrap_struct :: #force_inline proc(s: rawptr) -> Janet { return nanbox_wrap_c(s, .STRUCT) }
    wrap_tuple :: #force_inline proc(s: rawptr) -> Janet { return nanbox_wrap_c(s, .TUPLE) }
    wrap_fiber :: #force_inline proc(s: rawptr) -> Janet { return nanbox_wrap_(s, .FIBER) }
    wrap_array :: #force_inline proc(s: rawptr) -> Janet { return nanbox_wrap_(s, .ARRAY) }
    wrap_table :: #force_inline proc(s: rawptr) -> Janet { return nanbox_wrap_(s, .TABLE) }
    wrap_buffer :: #force_inline proc(s: rawptr) -> Janet { return nanbox_wrap_(s, .BUFFER) }
    wrap_string :: #force_inline proc(s: rawptr) -> Janet { return nanbox_wrap_c(s, .STRING) }
    wrap_symbol :: #force_inline proc(s: rawptr) -> Janet { return nanbox_wrap_c(s, .SYMBOL) }
    wrap_keyword :: #force_inline proc(s: rawptr) -> Janet { return nanbox_wrap_c(s, .KEYWORD) }
    wrap_abstract :: #force_inline proc(s: rawptr) -> Janet { return nanbox_wrap_(s, .ABSTRACT) }
    wrap_function :: #force_inline proc(s: rawptr) -> Janet { return nanbox_wrap_(s, .FUNCTION) }
    wrap_cfunction :: #force_inline proc(s: rawptr) -> Janet { return nanbox_wrap_(s, .CFUNCTION) }
    wrap_pointer :: #force_inline proc(s: rawptr) -> Janet { return nanbox_wrap_(s, .POINTER) }

/* Unwrap the pointer types */
    unwrap_struct :: #force_inline proc(x: Janet) -> Struct { return cast(Struct) nanbox_to_pointer(x) }
    unwrap_tuple :: #force_inline proc(x: Janet) ->Tuple { return Janet{pointer = nanbox_to_pointer(x)} }
    unwrap_fiber :: #force_inline proc(x: Janet) -> ^Fiber { return cast(^Fiber) nanbox_to_pointer(x) }
    unwrap_array :: #force_inline proc(x: Janet) -> ^Array {return cast(^Array) nanbox_to_pointer(x) }
    unwrap_table :: #force_inline proc(x: Janet) -> ^Table {return cast(^Table) nanbox_to_pointer(x) }
    unwrap_buffer :: #force_inline proc(x: Janet) -> ^Buffer {return cast(^Buffer) nanbox_to_pointer(x) }
    unwrap_string :: #force_inline proc(x: Janet) -> String {return cast(String) nanbox_to_pointer(x) }
    unwrap_symbol :: #force_inline proc(x: Janet) -> Symbol {return cast(Symbol) nanbox_to_pointer(x) }
    unwrap_keyword :: #force_inline proc(x: Janet) -> builtin.cstring { return (cast(builtin.cstring) nanbox_to_pointer(x)) }
    unwrap_abstract :: #force_inline proc(x: Janet) -> rawptr { return nanbox_to_pointer(x) }
    unwrap_pointer :: #force_inline proc(x: Janet) -> rawptr { return nanbox_to_pointer(x) }
    unwrap_function :: #force_inline proc(x: Janet) -> ^Function { return cast(^Function) nanbox_to_pointer(x) }
    unwrap_cfunction :: #force_inline proc(x: Janet) -> CFunction { return cast(CFunction) nanbox_to_pointer(x) }

    @(link_prefix="janet_")
    @(default_calling_convention="c")
    foreign janet {

        /* Use JANET_API so that modules will use a local version of these functions if possible */
        nanbox_to_pointer :: proc(x: Janet) -> rawptr ---
        nanbox_from_pointer :: proc(p: rawptr, tagmask: u64) -> Janet ---
        nanbox_from_cpointer :: proc(p: rawptr, tagmask: u64) -> Janet ---
        nanbox_from_double :: proc(d: f64) -> Janet ---
        nanbox_from_bits :: proc(bits: u64) -> Janet ---
    }

} else do #panic("Only implemented Nanbox64")

wrap_integer :: #force_inline proc(x: i32) -> Janet { return wrap_number( transmute(f64) [2]i32{0,x} ) }
unwrap_integer :: #force_inline proc(x: Janet) -> i32 { return transmute(i32) ([2]i32)(unwrap_number(x))[0] }

/* Some function definition flags */
JANET_FUNCDEF_FLAG_VARARG       :: 0x10000
JANET_FUNCDEF_FLAG_NEEDSENV     :: 0x20000
JANET_FUNCDEF_FLAG_HASSYMBOLMAP :: 0x40000
JANET_FUNCDEF_FLAG_HASNAME      :: 0x80000
JANET_FUNCDEF_FLAG_HASSOURCE    :: 0x100000
JANET_FUNCDEF_FLAG_HASDEFS      :: 0x200000
JANET_FUNCDEF_FLAG_HASENVS      :: 0x400000
JANET_FUNCDEF_FLAG_HASSOURCEMAP :: 0x800000
JANET_FUNCDEF_FLAG_STRUCTARG    :: 0x1000000
JANET_FUNCDEF_FLAG_HASCLOBITSET :: 0x2000000
JANET_FUNCDEF_FLAG_TAG          :: 0xFFFF


ParserStatus :: enum {
    ROOT,
    ERROR,
    PENDING,
    DEAD
}

/*opaque pointer*/
ParseState :: struct {}

/* A janet parser */
Parser :: struct {
    args: [^]Janet,
    error: builtin.cstring,
    states: [^]ParseState,
    buf: [^]u8,
    argcount: uint,
    argcap: uint,
    statecount: uint,
    statecap: uint,
    bufcount: uint,
    bufcap: uint,
    line: uint,
    column: uint,
    pending: uint,
    lookback: i32,
    flag: i32,
}

/* A context for marshaling and unmarshaling abstract types */
MarshalContext :: struct {
    m_state: rawptr,
    u_state: rawptr,
    flags: i32,
    data: [^]u8,
    at: ^AbstractType
}

File :: struct {
    file: libc.FILE,
    flags: i32,
}

/* For janet_try and janet_restore */
TryState :: struct {
    /* old state */
    stackn: i32,
    gc_handle: i32,
    vm_fiber: ^Fiber,
    vm_jmp_buf: ^libc.jmp_buf,
    vm_return_reg: ^Janet,
    /* new state */
    buf: libc.jmp_buf,
    payload: Janet,
    coerce_error: i32,
}

/***** START SECTION OPCODES *****/

/* Bytecode op argument types */
OpArgType :: enum {
    SLOT,
    ENVIRONMENT,
    CONSTANT,
    INTEGER,
    TYPE,
    SIMPLETYPE,
    LABEL,
    FUNCDEF
}

/* Various types of instructions */
InstructionType :: enum {
    ZERO, /* No args */
    S, /* Slot(3) */
    L, /* Label(3) */
    SS, /* Slot(1), Slot(2) */
    SL, /* Slot(1), Label(2) */
    ST, /* Slot(1), Slot(2) */
    SI, /* Slot(1), Immediate(2) */
    SD, /* Slot(1), Closure(2) */
    SU, /* Slot(1), Unsigned Immediate(2) */
    SSS, /* Slot(1), Slot(1), Slot(1) */
    SSI, /* Slot(1), Slot(1), Immediate(1) */
    SSU, /* Slot(1), Slot(1), Unsigned Immediate(1) */
    SES, /* Slot(1), Environment(1), Far Slot(1) */
    SC /* Slot(1), Constant(2) */
}

/* All opcodes for the bytecode interpreter. */
OpCode :: enum {
    NOOP,
    ERROR,
    TYPECHECK,
    RETURN,
    RETURN_NIL,
    ADD_IMMEDIATE,
    ADD,
    SUBTRACT_IMMEDIATE,
    SUBTRACT,
    MULTIPLY_IMMEDIATE,
    MULTIPLY,
    DIVIDE_IMMEDIATE,
    DIVIDE,
    DIVIDE_FLOOR,
    MODULO,
    REMAINDER,
    BAND,
    BOR,
    BXOR,
    BNOT,
    SHIFT_LEFT,
    SHIFT_LEFT_IMMEDIATE,
    SHIFT_RIGHT,
    SHIFT_RIGHT_IMMEDIATE,
    SHIFT_RIGHT_UNSIGNED,
    SHIFT_RIGHT_UNSIGNED_IMMEDIATE,
    MOVE_FAR,
    MOVE_NEAR,
    JUMP,
    JUMP_IF,
    JUMP_IF_NOT,
    JUMP_IF_NIL,
    JUMP_IF_NOT_NIL,
    GREATER_THAN,
    GREATER_THAN_IMMEDIATE,
    LESS_THAN,
    LESS_THAN_IMMEDIATE,
    EQUALS,
    EQUALS_IMMEDIATE,
    COMPARE,
    LOAD_NIL,
    LOAD_TRUE,
    LOAD_FALSE,
    LOAD_INTEGER,
    LOAD_CONSTANT,
    LOAD_UPVALUE,
    LOAD_SELF,
    SET_UPVALUE,
    CLOSURE,
    PUSH,
    PUSH_2,
    PUSH_3,
    PUSH_ARRAY,
    CALL,
    TAILCALL,
    RESUME,
    SIGNAL,
    PROPAGATE,
    IN,
    GET,
    PUT,
    GET_INDEX,
    PUT_INDEX,
    LENGTH,
    MAKE_ARRAY,
    MAKE_BUFFER,
    MAKE_STRING,
    MAKE_STRUCT,
    MAKE_TABLE,
    MAKE_TUPLE,
    MAKE_BRACKET_TUPLE,
    GREATER_THAN_EQUAL,
    LESS_THAN_EQUAL,
    NEXT,
    NOT_EQUALS,
    NOT_EQUALS_IMMEDIATE,
    CANCEL,
    INSTRUCTION_COUNT
}

CompileStatus :: enum {
    OK,
    ERROR
}

CompileResult :: struct {
    funcdef: ^FuncDef,
    error: String,
    macrofiber: ^Fiber,
    error_mapping: SourceMapping,
    status: CompileStatus,
}


/* Tuple */
JANET_TUPLE_FLAG_BRACKETCTOR  :: 0x10000
tuple_from_head :: #force_inline proc(gcobject: ^GCObject) -> Tuple {
    return cast(Tuple) Janet{pointer = rawptr(uintptr(gcobject) + offset_of(TupleHead, data))}
}
tuple_length :: #force_inline proc(t: Tuple) -> i32 { return tuple_head(t).length }
tuple_hash :: #force_inline proc(t: Tuple) -> i32 { return tuple_head(t).hash }
tuple_sm_line :: #force_inline proc(t: Tuple) -> i32 { return tuple_head(t).sm_line }
tuple_sm_column :: #force_inline proc(t: Tuple) -> i32 { return tuple_head(t).sm_column }
tuple_flag :: #force_inline proc(t: Tuple) -> i32  { return tuple_head(t).gc.flags }


/* String/Symbol functions */
string_length :: #force_inline proc(s: String) -> i32 { return string_head(s).length }
string_hash   :: #force_inline proc(s: String) -> i32 { return string_head(s).hash }

cstringv :: #force_inline proc(cstr: builtin.cstring) -> Janet { return wrap_string(cast(rawptr) cstring(cstr)) }
stringv :: #force_inline proc(str: [^]u8, len: i32) -> Janet { return wrap_string(cast(rawptr) string(str, len)) }

/* Symbol */

symbolv  :: #force_inline proc(str: builtin.cstring, len: i32) -> Janet { return wrap_symbol(cast(rawptr) symbol(str, len)) }
csymbolv :: #force_inline proc(cstr: builtin.cstring)          -> Janet { return wrap_symbol(cast(rawptr) csymbol(cstr)) }

/* Keyword functions */
keyword :: symbol
ckeyword :: csymbol
keywordv :: #force_inline proc(str: builtin.cstring, len: i32) -> Janet { return wrap_keyword(rawptr(keyword(str, len))) }
ckeywordv :: #force_inline proc(cstr: builtin.cstring) -> Janet { return wrap_keyword(rawptr(ckeyword(cstr))) }

/* Structs */
struct_head :: #force_inline proc(t: ^KV) -> ^StructHead { return cast(^StructHead) (uintptr(t) - offset_of(StructHead, data)) }
struct_from_head :: #force_inline proc(gcobject: ^GCObject) -> Struct { return cast(Struct) rawptr(uintptr(gcobject) + offset_of(StructHead, data)) }


struct_length :: #force_inline proc(t: Struct) -> i32 { return struct_head(t).length }
struct_capacity :: #force_inline proc(t: Struct) -> i32 { return struct_head(t).capacity }
struct_hash :: #force_inline proc(t: Struct) -> i32 { return struct_head(t).hash }
struct_proto :: #force_inline proc(t: Struct) -> ^KV { return struct_head(t).proto }

/* Abstract */
abstract_head      :: #force_inline proc(u: rawptr) -> ^AbstractHead { return cast(^AbstractHead) rawptr(uintptr(u) - offset_of(AbstractHead, data)) }
abstract_from_head :: #force_inline proc(gcobject: ^GCObject) ->  Abstract { return cast(Abstract) (uintptr(gcobject) + offset_of(AbstractHead, data)) }
abstract_type      :: #force_inline proc(u: rawptr) -> ^AbstractType { return abstract_head(u).type }
abstract_size      :: #force_inline proc(u: rawptr) -> uint { return abstract_head(u).size }

/* Native */
Module  :: #type proc "c" (t: ^Table)
Modconf :: #type proc "c" () -> BuildConfig


/* Marshaling */
MARSHAL_UNSAFE    :: 0x20000
MARSHAL_NO_CYCLES :: 0x40000

/* Pretty printing */
PRETTY_COLOR   :: 1
PRETTY_ONELINE :: 2
PRETTY_NOTRUNC :: 4

/* Misc */
try :: #force_inline proc(state: ^TryState) -> Signal {
    try_init(state)
    return Signal(libc.setjmp(&state.buf))
}

/* Sandboxing API */
Sandbox_Flag :: enum u32 {
    SANDBOX = 1,
    SUBPROCESS ,
    NET_CONNECT,
    NET_LISTEN,
    FFI_DEFINE,
    FS_WRITE,
    FS_READ,
    HRTIME,
    ENV,
    DYNAMIC_MODULES,
    FS_TEMP,
    FFI_USE,
    FFI_JIT,
    SIGNAL,
}
Sandbox_Flags :: bit_set[Sandbox_Flag]
SANDBOX_FFI :: Sandbox_Flags{ .FFI_DEFINE, .FFI_USE, .FFI_JIT }
SANDBOX_FS  :: Sandbox_Flags{ .FS_WRITE, .FS_READ, .FS_TEMP }
SANDBOX_NET :: Sandbox_Flags{ .NET_CONNECT, .NET_LISTEN }

SANDBOX_ALL :: ~Sandbox_Flags{}

/* Scratch Memory API */
ScratchFinalizer :: #type proc(rawptr)

/* C Library helpers */
BindingType :: enum {
    NONE,
    DEF,
    VAR,
    MACRO,
    DYNAMIC_DEF,
    DYNAMIC_MACRO
}

Binding :: struct {
    type: BindingType,
    value: Janet,
    deprecation: enum {
        NONE,
        RELAXED,
        NORMAL,
        STRICT,
    },
}

printf :: #force_inline proc(format: builtin.cstring, x: ..any) { dynprintf("out", libc.stdout, format, x) }
eprintf :: #force_inline proc(format: builtin.cstring, x: ..any) { dynprintf("err", libc.stderr, format, x) }

File_Flag :: enum {
    WRITE = 1,
    READ,
    APPEND,
    UPDATE,
    NOT_CLOSEABLE,
    CLOSED,
    BINARY,
    SERIALIZABLE,
    NONIL,
}

File_Flags :: bit_set[File_Flag]

when PEG {
    PegOpCod :: enum {
        LITERAL,      /* [len, bytes...] */
        NCHAR,        /* [n] */
        NOTNCHAR,     /* [n] */
        RANGE,        /* [lo | hi << 16 (1 word)] */
        SET,          /* [bitmap (8 words)] */
        LOOK,         /* [offset, rule] */
        CHOICE,       /* [len, rules...] */
        SEQUENCE,     /* [len, rules...] */
        IF,           /* [rule_a, rule_b (b if a)] */
        IFNOT,        /* [rule_a, rule_b (b if not a)] */
        NOT,          /* [rule] */
        BETWEEN,      /* [lo, hi, rule] */
        GETTAG,       /* [searchtag, tag] */
        CAPTURE,      /* [rule, tag] */
        POSITION,     /* [tag] */
        ARGUMENT,     /* [argument-index, tag] */
        CONSTANT,     /* [constant, tag] */
        ACCUMULATE,   /* [rule, tag] */
        GROUP,        /* [rule, tag] */
        REPLACE,      /* [rule, constant, tag] */
        MATCHTIME,    /* [rule, constant, tag] */
        ERROR,        /* [rule] */
        DROP,         /* [rule] */
        BACKMATCH,    /* [tag] */
        TO,           /* [rule] */
        THRU,         /* [rule] */
        LENPREFIX,    /* [rule_a, rule_b (repeat rule_b rule_a times)] */
        READINT,      /* [(signedness << 4) | (endianness << 5) | bytewidth, tag] */
        LINE,         /* [tag] */
        COLUMN,       /* [tag] */
        UNREF,        /* [rule, tag] */
        CAPTURE_NUM,  /* [rule, tag] */
        SUB,          /* [rule, rule] */
        TIL,          /* [rule, rule] */
        SPLIT,        /* [rule, rule] */
        NTH,          /* [nth, rule, tag] */
        ONLY_TAGS,    /* [rule] */
    }

    Peg :: struct {
        bytecode: [^]u32,
        constants: [^]Janet,
        bytecode_len: uint,
        num_constants: u32,
        has_backref: bool,
    }

}

@(default_calling_convention="c")
foreign janet {

    @(private) janet_atomic_inc          :: proc(x: ^AtomicInt) -> AtomicInt ---
    @(private) janet_atomic_dec          :: proc(x: ^AtomicInt) -> AtomicInt ---
    @(private) janet_atomic_load         :: proc(x: ^AtomicInt) -> AtomicInt ---
    @(private) janet_atomic_load_relaxed :: proc(x: ^AtomicInt) -> AtomicInt ---
}

atomic_inc          :: #force_inline proc(x: ^AtomicInt) -> AtomicInt { x := intrinsics.volatile_load(x); return janet_atomic_inc(&x) }
atomic_dec          :: #force_inline proc(x: ^AtomicInt) -> AtomicInt { x := intrinsics.volatile_load(x); return janet_atomic_dec(&x) }
atomic_load         :: #force_inline proc(x: ^AtomicInt) -> AtomicInt { x := intrinsics.volatile_load(x); return janet_atomic_load(&x)}
atomic_load_relaxed :: #force_inline proc(x: ^AtomicInt) -> AtomicInt { x := intrinsics.volatile_load(x); return janet_atomic_load_relaxed(&x) }


@(link_prefix="janet_", default_calling_convention="c")
foreign janet {

    type_names:   [16][^]u8
    signal_names: [14][^]u8
    status_names: [16][^]u8

    instructions: [int(OpCode(.INSTRUCTION_COUNT))]InstructionType



    // struct_head   :: proc(st: Struct)       -> StructHead ---
    // abstract_head :: proc(abstract: rawptr) -> AbstractHead ---
    string_head   :: proc(s: String)        -> ^StringHead ---
    tuple_head    :: proc(tuple: Tuple)     -> ^TupleHead ---


    checkint16    :: proc(x: Janet)                                -> bool ---
    checkuint16   :: proc(x: Janet)                                -> bool ---
    checkint      :: proc(x: Janet)                                -> bool ---
    checkuint     :: proc(x: Janet)                                -> bool ---
    checkint64    :: proc(x: Janet)                                -> bool ---
    checkuint64   :: proc(x: Janet)                                -> bool ---
    checksize     :: proc(x: Janet)                                -> bool ---
    checkabstract :: proc(x: Janet, at: ^AbstractType)             -> Abstract ---

    /* Parsing */
    parser_type: AbstractType
    parser_init            :: proc(parser: ^Parser) ---
    parser_deinit          :: proc(parser: ^Parser) ---
    parser_consume         :: proc(parser: ^Parser, c: u8) ---
    parser_status          :: proc(parser: ^Parser)        -> ParserStatus ---
    parser_produce         :: proc(parser: ^Parser)        -> Janet ---
    parser_produce_wrapped :: proc(parser: ^Parser)        -> Janet ---
    parser_error           :: proc(parser: ^Parser)        -> builtin.cstring ---
    parser_flush           :: proc(parser: ^Parser) ---
    parser_eof             :: proc(parser: ^Parser) ---
    parser_has_more        :: proc(parser: ^Parser)        -> i32 ---


    /* Compilation */
    compile      :: proc(source: Janet, env: ^Table, _where: String)                -> CompileResult ---
    compile_lint :: proc(source: Janet, env: ^Table, _where: String, lints: ^Array) -> CompileResult ---

    /* Get the default environment for janet */
    core_env          :: proc(replacements: ^Table) -> ^Table ---
    core_lookup_table :: proc(replacements: ^Table) -> ^Table ---

    /* Execute strings */
    dobytes  :: proc(env: ^Table, bytes: [^]u8, len: i32, sourcePath: builtin.cstring, out: ^Janet) -> i32 ---
    dostring :: proc(env: ^Table, str: builtin.cstring, sourcePath: builtin.cstring, out: ^Janet)           -> i32 ---

    /* Run the entrypoint of a wrapped program */
    loop_fiber :: proc(fiber: ^Fiber) -> i32 ---

    /* Number scanning */
    scan_number      :: proc(str: builtin.cstring, len: i32, out: ^f64)           -> i32 ---
    scan_number_base :: proc(str: builtin.cstring, len: i32, base: i32, out: f64) -> i32 ---
    scan_int64       :: proc(str: builtin.cstring, len: i32, out: ^i64)           -> i32 ---
    scan_uint64      :: proc(str: builtin.cstring, len: i32, out: ^u64)           -> i32 ---
    scan_numeric     :: proc(str: builtin.cstring, len: i32, out: ^Janet)         -> i32 ---

    /* Debugging */
    debug_break :: proc(def: ^FuncDef, pc: i32) ---
    debug_unbreak :: proc(def: ^FuncDef, pc: i32) ---
    debug_find :: proc(def_out: ^^FuncDef, pc_out: ^i32, source: String, line: i32, column: i32) ---

    /* RNG */
    rng_type: AbstractType
    default_rng :: proc() -> ^RNG ---
    rng_seed :: proc(rng: ^RNG, seed: u32) ---
    rng_longseed :: proc(rng: ^RNG, bytes: [^]u8, len: i32) ---
    rng_u32 :: proc(rng: ^RNG) -> u32 ---
    rng_double :: proc(rng: ^RNG) -> f64 ---

    /* Array functions */
    array          :: proc(capacity: i32)              -> ^Array ---
    array_weak     :: proc(capacity: i32)              -> ^Array ---
    array_n        :: proc(elements: [^]Janet, n: i32) -> ^Array ---
    array_pop      :: proc(array: ^Array)              -> Janet ---
    array_peek     :: proc(array: ^Array)              -> Janet ---
    array_ensure   :: proc(array: ^Array, capacity: i32, growth: i32) ---
    array_setcount :: proc(array: ^Array, count: i32) ---
    array_push     :: proc(array: ^Array, x: Janet) ---


    /* Buffer functions */
    buffer                :: proc(capacity: i32)                             -> ^Buffer ---
    buffer_init           :: proc(buffer: ^Buffer, capacity: i32)            -> ^Buffer ---
    pointer_buffer_unsafe :: proc(memory: rawptr, capacity: i32, count: i32) -> ^Buffer ---
    buffer_deinit         :: proc(buffer: ^Buffer) ---
    buffer_ensure         :: proc(buffer: ^Buffer, capacity: i32, growth: i32) ---
    buffer_setcount       :: proc(buffer: ^Buffer, count: i32) ---
    buffer_extra          :: proc(buffer: ^Buffer, n: i32) ---
    buffer_push_bytes     :: proc(buffer: ^Buffer, str: [^]u8, len: i32) ---
    buffer_push_string    :: proc(buffer: ^Buffer, string: String) ---
    buffer_push_cstring   :: proc(buffer: ^Buffer, cstr: builtin.cstring) ---
    buffer_push_u8        :: proc(buffer: ^Buffer, x: u8) ---
    buffer_push_u16       :: proc(buffer: ^Buffer, x: u16) ---
    buffer_push_u32       :: proc(buffer: ^Buffer, x: u32) ---
    buffer_push_u64       :: proc(buffer: ^Buffer, x: u64) ---

    /* Tuple */
    tuple_begin :: proc(length: i32)              -> ^Janet ---
    tuple_end   :: proc(tuple: ^Janet)            -> Tuple ---
    tuple_n     :: proc(values: [^]Janet, n: i32) -> Tuple ---

    /* String/Symbol functions */
    string_begin :: proc(length: i32) -> [^]u8 ---
    string_end   :: proc(str: [^]u8) -> String ---
    string :: proc(buf: [^]u8, len: i32) -> String ---
    cstring :: proc(cstr: builtin.cstring) -> String ---
    string_compare :: proc(lhs, rhs: String) -> i32 ---
    string_equal :: proc(lhs, rhs: String) -> i32 ---
    string_equalconst :: proc(lhs: String, rhs: [^]u8, rlen: i32, rhash: i32) -> i32 ---
    description :: proc(x: Janet) -> String ---
    to_string :: proc(x: Janet) -> String ---
    to_string_b :: proc(buffer: ^Buffer, x: Janet) ---
    description_b :: proc(buffer: ^Buffer, x: Janet) ---
    formatc :: proc(format: builtin.cstring, #c_vararg args: ..any) -> String ---
    formatb :: proc(bufp: ^Buffer, format: builtin.cstring, #c_vararg args: ..any) -> ^Buffer ---
    formatbv :: proc(bufp: ^Buffer,format: builtin.cstring, args: c.va_list) ---

    /* Symbol functions */
    symbol :: proc(cstr: builtin.cstring, len: i32) -> Symbol ---
    csymbol :: proc(cstr: builtin.cstring) -> Symbol ---
    symbol_gen :: proc() -> Symbol ---

    /* Structs */
    struct_begin :: proc(count: i32) -> ^KV ---
    struct_put :: proc(st: ^KV, key, value: Janet) ---
    struct_end :: proc(st: ^KV) -> Struct  ---
    struct_get :: proc(st: Struct, key: Janet) -> Janet  ---
    struct_rawget :: proc(st: Struct, key: Janet) -> Janet  ---
    struct_get_ex :: proc(st: Struct, key: Janet, which: ^Struct) -> Janet  ---
    struct_to_table :: proc(st: Struct) -> ^Table  ---
    struct_find :: proc(st: Struct, key: Janet) -> ^KV ---

    /* Table functions */
    table :: proc(capacity: i32) -> ^Table ---
    table_init :: proc(table: ^Table, capacity: i32) -> ^Table ---
    table_init_raw :: proc(table: ^Table, capacity: i32) -> ^Table ---
    table_deinit :: proc(table: ^Table) ---
    table_get :: proc(t: ^Table, key: Janet) -> Janet ---
    table_get_ex :: proc(t: ^Table, key: Janet, which: ^^Table) -> Janet ---
    table_rawget :: proc(t: ^Table, key: Janet) -> Janet ---
    table_remove :: proc(t: ^Table, key: Janet) -> Janet ---
    table_put :: proc(t: ^Table, key, value: Janet) ---
    table_to_struct :: proc(t: ^Table) -> Struct ---
    table_merge_table :: proc(table, other: ^Table) ---
    table_merge_struct :: proc(table: ^Table, other: Struct) ---
    table_find :: proc(t: ^Table, key: Janet) -> ^KV ---
    table_clone :: proc(table: ^Table) -> ^Table ---
    table_clear :: proc(table: ^Table) ---
    table_weakk :: proc(capacity: i32) -> ^Table ---
    table_weakv :: proc(capacity: i32) -> ^Table ---
    table_weakkv :: proc(capacity: i32) -> ^Table ---

    /* Fiber */
    fiber            :: proc(callee: ^Function , capacity: i32, argc: i32, argv: [^]Janet) -> ^Fiber ---
    fiber_reset      :: proc(fiber: ^Fiber , callee: ^Function, argc: i32, argv: [^]Janet) -> ^Fiber ---
    fiber_status     :: proc(fiber: ^Fiber)                                                -> FiberStatus ---
    fiber_can_resume :: proc(fiber: ^Fiber)                                                -> i32 ---
    current_fiber    :: proc()                                                             -> ^Fiber ---
    root_fiber       :: proc()                                                             -> ^Fiber ---

    /* Treat similar types through uniform interfaces for iteration */
    indexed_view    :: proc(seq: Janet, data: [^]^Janet, len: ^i32)         -> i32 ---
    bytes_view      :: proc(str: Janet, data: [^]^u8, len: ^i32)            -> i32 ---
    dictionary_view :: proc(tab: Janet, data: [^]^KV, len: ^i32, cap: ^i32) -> i32 ---
    dictionary_get  :: proc(data: [^]KV, cap: i32, key: Janet)              -> Janet ---
    dictionary_next :: proc(kvs: [^]KV, cap: i32, kv: ^KV)                  -> ^KV ---

    /* Abstract */
    abstract_begin :: proc(type: ^AbstractType, size: i32) -> rawptr ---
    abstract_end   :: proc(abstractTemplate: rawptr)       -> Abstract ---
    abstract       :: proc(type: ^AbstractType, size: i32) -> Abstract --- /* begin and end in one call */

    /* Native */
    native :: proc(name: builtin.cstring, error: ^String) -> Module ---


    marshal         :: proc(buf: ^Buffer, x: Janet, rreg: ^Table,flags: i32) ---
    unmarshal       :: proc(bytes: [^]u8, len: uint, flags: i32, reg: ^Table, next: ^[^]u8) -> Janet ---
    env_lookup      :: proc(env: ^Table) -> ^Table ---
    env_lookup_into :: proc(renv: ^Table, env: ^Table, prefix: [^]u8, recurse: i32) ---

    /* GC */
    mark         :: proc(x: Janet) ---
    sweep        :: proc() ---
    collect      :: proc() ---
    clear_memory :: proc() ---
    gcroot       :: proc(root: Janet) ---
    gcunroot     :: proc(root: Janet) -> i32 ---
    gcunrootall  :: proc(root: Janet) -> i32 ---
    gclock       :: proc() -> i32 ---
    gcunlock     :: proc(handle: i32) ---
    gcpressure   :: proc(s: uint) ---

    /* Functions */
    funcdef_alloc :: proc() -> ^FuncDef ---
    thunk         :: proc(def: ^FuncDef) -> ^Function ---
    thunk_delay   :: proc(x: Janet) -> ^Function ---
    verify        :: proc(def: ^FuncDef) -> i32 ---

    /* Pretty printing */
    pretty :: proc(buffer: ^Buffer, depth: i32, flags: i32, x: Janet) -> ^Buffer ---


    /* Misc */
    try_init :: proc(state: ^TryState) ---

    restore          :: proc(state: ^TryState)  ---
    equals           :: proc(x, y: Janet) -> int ---
    hash             :: proc(x: Janet) -> i32 ---
    compare          :: proc(x, y: Janet) -> i32 ---
    cstrcmp          :: proc(str: String, other: builtin.cstring) -> i32 ---
    @(link_name="janet_in")
    in_              :: proc(ds, key: Janet) -> Janet ---
    get              :: proc(ds, key: Janet) -> Janet ---
    next             :: proc(ds, key: Janet) -> Janet ---
    getindex         :: proc(ds: Janet, index: i32) -> Janet ---
    length           :: proc(x: Janet) -> i32 ---
    lengthv          :: proc(x: Janet) -> Janet ---
    put              :: proc(ds, key, value: Janet)  ---
    putindex         :: proc(ds: Janet, index: i32, value: Janet)  ---
    wrap_number_safe :: proc(x: f64) -> Janet ---
    keyeq            :: proc(x: Janet, cstr: builtin.cstring) -> i32 ---
    streq            :: proc(x: Janet, cstr: builtin.cstring) -> i32 ---
    symeq            :: proc(x: Janet, cstr: builtin.cstring) -> i32 ---
    sorted_keys      :: proc(dict: ^KV, cap: i32, index_buffer: ^i32) -> i32 ---

    /* VM functions */
    init                          :: proc() -> i32 ---
    deinit                        :: proc() ---
    vm_alloc                      :: proc() -> ^VM ---
    local_vm                      :: proc() -> ^VM ---
    vm_free                       :: proc(vm: ^VM) ---
    vm_save                       :: proc(into: ^VM) ---
    vm_load                       :: proc(from: ^VM) ---
    interpreter_interrupt         :: proc(vm: ^VM) ---
    interpreter_interrupt_handled :: proc(vm: ^VM) ---
    @(link_name="janet_continue")
    continue_                     :: proc(fiber: ^Fiber, in_: Janet, out: ^Janet) -> Signal ---
    continue_signal               :: proc(fiber: ^Fiber, in_: Janet, out: ^Janet, sig: Signal) -> Signal ---
    pcall                         :: proc(fun: ^Function, argn: i32, argv: [^]Janet, out: ^Janet, f: ^^Fiber) -> Signal ---
    step                          :: proc(fiber: ^Fiber, in_: Janet, out: ^Janet) -> Signal ---
    call                          :: proc(fun: ^Function, argc: i32, argv: [^]Janet) -> Janet ---
    mcall                         :: proc(name: builtin.cstring, argc: i32, argv: [^]Janet) -> Janet ---
    stacktrace                    :: proc(fiber: ^Fiber, err: Janet) ---
    stacktrace_ext                :: proc(fiber: ^Fiber, err: Janet, prefix: builtin.cstring) ---

    /* Sandboxing API */
    sandbox        :: proc(flags: u32) ---
    sandbox_assert :: proc(forbidden_flags: u32) ---

    /* Scratch Memory API */
    smalloc    :: proc(size: uint) -> rawptr ---
    srealloc   :: proc(mem: rawptr, size: uint) -> rawptr ---
    scalloc    :: proc(nmemb: uint, size: uint) -> rawptr ---
    sfinalizer :: proc(mem: rawptr, finalizer: ScratchFinalizer) ---
    sfree      :: proc(mem: rawptr) ---

    /* C Library helpers */
    def          :: proc(env: ^Table, name: builtin.cstring, val: Janet, documentation: builtin.cstring) ---
    var          :: proc(env: ^Table, name: builtin.cstring, val: Janet, documentation: builtin.cstring) ---
    cfuns        :: proc(env: ^Table, regprefix: builtin.cstring, cfuns: [^]Reg) ---
    cfuns_prefix :: proc(env: ^Table, regprefix: builtin.cstring, cfuns: [^]Reg) ---
    resolve      :: proc(env: ^Table, sym: Symbol, out: ^Janet) -> BindingType ---
    resolve_ext  :: proc(env: ^Table, sym: Symbol) -> Binding ---

    /* Get values from the core environment. */
    resolve_core :: proc(name: builtin.cstring) -> Janet ---

    //HACK: skipping from line 2010 to 2069, I have no idea how to do it

    /* Define things with source mapping information */
    cfuns_ext :: proc(env: ^Table, regprefix: builtin.cstring, cfuns: [^]RegExt) ---
    cfuns_ext_prefix :: proc(env: ^Table, regprefix: builtin.cstring, cfuns: [^]RegExt) ---
    def_sm           :: proc(env: ^Table,
                             name: builtin.cstring,
                             val: Janet,
                             documentation: builtin.cstring,
                             source_file: builtin.cstring,
                             source_line: i32) ---
    var_sm           :: proc(env: ^Table,
                             name: builtin.cstring,
                             val: Janet,
                             documentation: builtin.cstring,
                             source_file: builtin.cstring,
                             source_line: i32) ---

    /* Legacy definition of C functions */
    register :: proc(name: builtin.cstring, cfun: CFunction) ---

    //NOTE: skipping from 2078 to 2094, this should be useless by virtue of not using C and just linking

    signalv        :: proc(signal: Signal, message: Janet) -> ! ---
    panicv         :: proc(message: Janet) -> ! ---
    panic          :: proc(message: builtin.cstring) -> ! ---
    panics         :: proc(message: String) -> ! ---
    panicf         :: proc(format: builtin.cstring, #c_vararg x: ..any) -> ! ---
    dynprintf      :: proc(name: builtin.cstring, dflt_file: ^c.FILE, format: builtin.cstring, #c_vararg x: ..any) ---
    panic_type     :: proc(x: Janet, n: i32, expected: i32) -> ! ---
    panic_abstract :: proc(x: Janet, n:i32 , at: ^AbstractType) -> ! ---
    arity          :: proc(arity, min, max: i32) ---
    fixarity       :: proc(arity, fix: i32) ---

    getmethod  :: proc(method: Keyword, methods: [^]Method, out: ^Janet) -> i32 ---
    nextmethod :: proc(methods: [^]Method, key: Janet) -> Janet ---

    getnumber    :: proc(argv: ^Janet, n: i32) -> f64 ---
    getarray     :: proc(argv: ^Janet, n: i32) -> ^Array ---
    gettuple     :: proc(argv: ^Janet, n: i32) -> Tuple ---
    gettable     :: proc(argv: ^Janet, n: i32) -> ^Table ---
    getstruct    :: proc(argv: ^Janet, n: i32) -> Struct ---
    getstring    :: proc(argv: ^Janet, n: i32) -> String ---
    getcstring   :: proc(argv: ^Janet, n: i32) -> builtin.cstring ---
    getcbytes    :: proc(argv: ^Janet, n: i32) -> [^]u8 ---
    getsymbol    :: proc(argv: ^Janet, n: i32) -> Symbol ---
    getkeyword   :: proc(argv: ^Janet, n: i32) -> Keyword ---
    getbuffer    :: proc(argv: ^Janet, n: i32) -> ^Buffer ---
    getfiber     :: proc(argv: ^Janet, n: i32) -> ^Fiber ---
    getfunction  :: proc(argv: ^Janet, n: i32) -> ^Function ---
    getcfunction :: proc(argv: ^Janet, n: i32) -> CFunction ---
    getboolean   :: proc(argv: ^Janet, n: i32) -> i32 ---
    getpointer   :: proc(argv: ^Janet, n: i32) -> rawptr ---

    getnat :: proc(argv: [^]Janet, n: i32) -> i32 ---
    getinteger :: proc(argv: [^]Janet, n: i32) -> i32 ---
    getinteger16 :: proc(argv: [^]Janet, n: i32) -> i16 ---
    getinteger64 :: proc(argv: [^]Janet, n: i32) -> i64 ---
    getuinteger :: proc(argv: [^]Janet, n: i32) -> u32 ---
    getuinteger16 :: proc(argv: [^]Janet, n: i32) -> u16 ---
    getuinteger64 :: proc(argv: [^]Janet, n: i32) -> u64 ---
    getsize :: proc(argv: [^]Janet, n: i32) -> uint ---
    getindexed :: proc(argv: [^]Janet, n: i32) -> View ---
    getbytes :: proc(argv: [^]Janet, n: i32) -> ByteView ---
    getdictionary :: proc(argv: [^]Janet, n: i32) -> DictView ---
    getabstract :: proc(argv: [^]Janet, n: i32, at: ^AbstractType) -> rawptr ---
    getslice :: proc(argc: i32, argv: [^]Janet) -> Range ---
    gethalfrange :: proc(argv: [^]Janet, n: i32, length: i32, which: [^]u8) -> i32 ---
    getstartrange :: proc(argv: [^]Janet, argc: i32, n: i32,  length: i32) -> i32 ---
    getendrange :: proc(argv: [^]Janet, argc: i32, n: i32,  length: i32) -> i32 ---
    getargindex :: proc(argv: [^]Janet, n: i32, length: i32, which: [^]u8) -> i32 ---
    getflags :: proc(argv: [^]Janet, n: i32, flags: [^]u8) -> u64 ---

    /* Optionals */
    optnumber    :: proc(argv: [^]Janet, argc: i32, n: i32, dflt: f64) -> f64 ---
    opttuple     :: proc(argv: [^]Janet, argc: i32, n: i32, dflt: Tuple) -> Tuple ---
    optstruct    :: proc(argv: [^]Janet, argc: i32, n: i32, dflt: Struct) -> Struct ---
    optstring    :: proc(argv: [^]Janet, argc: i32, n: i32, dflt: String) -> String ---
    optcstring   :: proc(argv: [^]Janet, argc: i32, n: i32, dflt: builtin.cstring) -> builtin.cstring ---
    optcbytes    :: proc(argv: [^]Janet, argc: i32, n: i32, dflt: [^]u8) -> [^]u8 ---
    optsymbol    :: proc(argv: [^]Janet, argc: i32, n: i32, dflt: String) -> Symbol ---
    optkeyword   :: proc(argv: [^]Janet, argc: i32, n: i32, dflt: String) -> Keyword ---
    optfiber     :: proc(argv: [^]Janet, argc: i32, n: i32, dflt: ^Fiber) -> ^Fiber ---
    optfunction  :: proc(argv: [^]Janet, argc: i32, n: i32, dflt: ^Function) -> ^Function ---
    optcfunction :: proc(argv: [^]Janet, argc: i32, n: i32, dflt: CFunction) -> CFunction ---
    optboolean   :: proc(argv: [^]Janet, argc: i32, n: i32, dflt: bool) -> bool ---
    optpointer   :: proc(argv: [^]Janet, argc: i32, n: i32, dflt: rawptr) -> rawptr ---
    optnat       :: proc(argv: [^]Janet, argc: i32, n: i32, dflt: i32) -> i32 ---
    optinteger   :: proc(argv: [^]Janet, argc: i32, n: i32, dflt: i32) -> i32 ---
    optinteger64 :: proc(argv: [^]Janet, argc: i32, n: i32, dflt: i64) -> i64 ---
    optsize      :: proc(argv: [^]Janet, argc: i32, n: i32, dflt: uint) -> uint ---
    optabstract  :: proc(argv: [^]Janet, argc: i32, n: i32, at: ^AbstractType, dflt: Abstract) -> Abstract ---

    /* Mutable optional types specify a size default, and construct a new value if none is provided */
    optbuffer :: proc(argv: ^Janet, argc: i32, n: i32, dflt_len: i32) -> ^Buffer ---
    opttable :: proc(argv: ^Janet, argc: i32, n: i32, dflt_len: i32) -> ^Table ---
    optarray :: proc(argv: ^Janet, argc: i32, n: i32, dflt_len: i32) -> ^Array ---

    dyn    :: proc(name: builtin.cstring) -> Janet ---
    setdyn :: proc(name: builtin.cstring, value: Janet) ---

    file_type: AbstractType

    makefile   :: proc(f: ^libc.FILE, flags: i32) -> Janet ---
    makejfile  :: proc(f: ^libc.FILE, flags: i32) -> ^File ---
    getfile    :: proc(argv: [^]Janet, n: i32, flags: ^i32) -> ^libc.FILE ---
    dynfile    :: proc(name: builtin.cstring, def: ^libc.FILE) -> ^libc.FILE ---
    getjfile   :: proc(argv: [^]Janet, n: i32) -> ^File ---
    checkfile  :: proc(j: Janet) -> Abstract ---
    unwrapfile :: proc(j: Janet, flags: ^i32) -> libc.FILE ---
    file_close :: proc(file: ^File) -> i32 ---

    cryptorand :: proc(out: [^]u8, n: uint) -> i32 ---

    /* Marshal API */
    marshal_size     :: proc(ctx: ^MarshalContext, value: uint) ---
    marshal_int      :: proc(ctx: ^MarshalContext, value: i32) ---
    marshal_int64    :: proc(ctx: ^MarshalContext, value: i64) ---
    marshal_ptr      :: proc(ctx: ^MarshalContext, value: rawptr) ---
    marshal_byte     :: proc(ctx: ^MarshalContext, value: u8) ---
    marshal_bytes    :: proc(ctx: ^MarshalContext, bytes: [^]u8, len: uint) ---
    marshal_janet    :: proc(ctx: ^MarshalContext, x: Janet) ---
    marshal_abstract :: proc(ctx: ^MarshalContext, abstract: Abstract) ---

    unmarshal_ensure            :: proc(ctx: ^MarshalContext, size: uint) ---
    unmarshal_size              :: proc(ctx: ^MarshalContext) -> uint ---
    unmarshal_int               :: proc(ctx: ^MarshalContext) -> i32 ---
    unmarshal_int64             :: proc(ctx: ^MarshalContext) -> i64 ---
    unmarshal_ptr               :: proc(ctx: ^MarshalContext) -> rawptr ---
    unmarshal_byte              :: proc(ctx: ^MarshalContext) -> u8 ---
    unmarshal_bytes             :: proc(ctx: ^MarshalContext, dest: [^]u8, len: uint) ---
    unmarshal_janet             :: proc(ctx: ^MarshalContext) -> Janet  ---
    unmarshal_abstract          :: proc(ctx: ^MarshalContext, size: uint) -> Abstract  ---
    unmarshal_abstract_threaded :: proc(ctx: ^MarshalContext, size: uint) -> Abstract  ---
    unmarshal_abstract_reuse    :: proc(ctx: ^MarshalContext, p: rawptr) ---

    register_abstract_type :: proc(at: ^AbstractType) ---
    get_abstract_type      :: proc(key: Janet) -> [^]AbstractType ---

}




when JANET_EV {

    @(private)
    Janet_Ev :: struct {
        sched_id: u32, /* Increment everytime fiber is scheduled by event loop */
        ev_callback: EVCallback, /* Call this before starting scheduled fibers */
        ev_stream: ^Stream, /* which stream we are waiting on */
        ev_state: rawptr, /* Extra data for ev callback state. On windows, first element must be OVERLAPPED. */
        supervisor_channel: rawptr, /* Channel to push self to when complete */
    }

    OSMutex :: struct {}
    OSRWLock :: struct {}
    Channel :: struct {}

    stream_type: AbstractType
    channel_type: AbstractType

    STREAM_CLOSED        :: 0x1
    STREAM_SOCKET        :: 0x2
    STREAM_UNREGISTERED  :: 0x4
    STREAM_READABLE      :: 0x200
    STREAM_WRITABLE      :: 0x400
    STREAM_ACCEPTABLE    :: 0x800
    STREAM_UDPSERVER     :: 0x1000
    STREAM_NOT_CLOSEABLE :: 0x2000
    STREAM_TOCLOSE       :: 0x10000

    AsyncEvent :: enum {
        INIT = 0,
        MARK = 1,
        DEINIT = 2,
        CLOSE = 3,
        ERR = 4,
        HUP = 5,
        READ = 6,
        WRITE = 7,
        COMPLETE = 8, /* Used on windows for IOCP */
        FAILED = 9 /* Used on windows for IOCP */
    }

    AsyncMode :: enum {
        LISTEN_READ = 1,
        LISTEN_WRITE,
        LISTEN_BOTH
    }

    /* Wrapper around file descriptors and HANDLEs that can be polled. */
    Stream :: struct {
        handle: Handle,
        flags: u32,
        index: u32,
        read_fiber: ^Fiber,
        write_fiber: ^Fiber,
        methods: ^rawptr /* Methods for this stream */
    }

    EVCallback :: #type proc "c" (fiber: ^Fiber, event: AsyncEvent)


    /* Async service for calling a function or syscall in a background thread. This is not
    * as efficient in the slightest as using Streams but can be used for arbitrary blocking
    * functions and syscalls. */

    /* Used to pass data between the main thread and worker threads for simple tasks.
    * We could just use a pointer but this prevents malloc/free in the common case
    * of only a handful of arguments. */
    EVGenericMessage :: struct {
        tag: i32,
        argi: i32,
        argp: rawptr,
        argj: Janet,
        fiber: ^Fiber,
    }

    /* How to resume or cancel after a threaded call. Not exhaustive of the possible
    * ways one might want to resume after returning from a threaded call, but should
    * cover most of the common cases. For something more complicated, such as resuming
    * with an abstract type or a struct, one should use janet_ev_threaded_call instead
    * of janet_ev_threaded_await with a custom callback. */

    TCTag :: enum {
        NIL = 0,       /* resume with nil */
        INTEGER,       /* resume with janet_wrap_integer(argi) */
        STRING,        /* resume with janet_cstringv((const char *) argp) */
        STRINGF,       /* resume with janet_cstringv((const char *) argp), then call free on argp. */
        KEYWORD,       /* resume with janet_ckeywordv((const char *) argp) */
        ERR_STRING,    /* cancel with janet_cstringv((const char *) argp) */
        ERR_STRINGF,   /* cancel with janet_cstringv((const char *) argp), then call free on argp. */
        ERR_KEYWORD,   /* cancel with janet_ckeywordv((const char *) argp) */
        BOOLEAN,       /* resume with janet_wrap_boolean(argi) */
    }

    /* Function pointer that is run in the thread pool */
    ThreadedSubroutine ::  #type proc(arguments: EVGenericMessage) -> EVGenericMessage

    /* Handler for events posted to the event loop */
    Callback :: #type proc(return_value: EVGenericMessage)

    /* Handler that is run in the main thread with the result of the JanetAsyncSubroutine (same as JanetCallback) */
    ThreadedCallback :: #type proc(return_value: EVGenericMessage)


    @(link_prefix="janet_")
    @(default_calling_convention="c")
    foreign janet {
        /* Start listening for events from a stream on the current root fiber. After
        * calling this, users should call janet_await() before returning from the
        * current C Function. This also will call janet_await.
        * mode is which events to listen for, and callback is the function pointer to
        * call when ever an event is sent from the event loop. state is an optional (can be NULL)
        * pointer to data allocated with janet_malloc. This pointer will be passed to callback as
        * fiber->ev_state. It will also be freed for you by the runtime when the event loop determines
        * it can no longer be referenced. On windows, the contents of state MUST contained an OVERLAPPED struct at the 0 offset. */

        async_start_fiber :: proc(fiber: ^Fiber,
                                  stream: ^Stream,
                                  mode: AsyncMode,
                                  callback: EVCallback,
                                  state: rawptr) ---

        async_start :: proc(stream: ^Stream,
                            mode: AsyncMode,
                            callback: EVCallback,
                            state: rawptr) -> ! ---

        /* Do not send any more events to the given callback. Call this after scheduling fiber to be resume
        * or canceled. */
        async_end :: proc(fiber: ^Fiber) ---

        /* Needed for windows to mark a fiber as waiting for an IOCP completion event. Noop on other platforms. */
        async_in_flight :: proc(fiber: ^Fiber) ---

        /* On some platforms, it is important to be able to control if a stream is edge-trigger or level triggered.
        * For example, a server that is accepting connections might want to be level triggered or edge-triggered
        * depending on expected service. */
        stream_edge_triggered :: proc(stream: ^Stream) ---
        stream_level_triggered:: proc(stream: ^Stream) ---


        /* Run the event loop */
        loop :: proc() ---
        /* Run the event loop, but allow for user scheduled interrupts triggered
        * by janet_loop1_interrupt being called in library code, a signal handler, or
        * another thread.
        *
        * Example:
        *
        * while (!janet_loop_done()) {
        *   // One turn of the event loop
        *   JanetFiber *interrupted_fiber = janet_loop1();
        *   // interrupted_fiber may be NULL
        *   // do some work here periodically...
        *   if (NULL != interrupted_fiber) {
        *     if (cancel_interrupted_fiber) {
        *       janet_cancel(interrupted_fiber, janet_cstringv("fiber was interrupted for [reason]"));
        *     } else {
        *       janet_schedule(interrupted_fiber, janet_wrap_nil());
        *     }
        *   }
        * }
        *
        */

        loop_done :: proc() -> i32 ---
        loop1 :: proc() -> ^Fiber ---
        loop1_interrupt :: proc(vm: ^VM) ---


        /* Wrapper around streams */
        stream :: proc(handle: Handle, flags: u32, methods: [^]Method) -> ^Stream ---
        stream_ext :: proc(handle: Handle, flags: u32, methods: [^]Method, size: uint) -> ^Stream  --- /* Allow for type punning streams */
        stream_close :: proc(stream: ^Stream) ---
        cfun_stream_close :: proc(argc: i32, argv: [^]Janet) -> Janet ---
        cfun_stream_read :: proc(argc: i32, argv: [^]Janet) -> Janet ---
        cfun_stream_chunk :: proc(argc: i32, argv: [^]Janet) -> Janet ---
        cfun_stream_write :: proc(argc: i32, argv: [^]Janet) -> Janet ---
        stream_flags :: proc(stream: ^Stream, flags: u32) ---

        /* Queue a fiber to run on the event loop */
        schedule :: proc(fiber: ^Fiber, value: Janet) ---
        cancel :: proc(fiber: ^Fiber, value: Janet) ---
        schedule_signal :: proc(fiber: ^Fiber, value: Janet, sig: Signal) ---
        schedule_soon :: proc(fiber: ^Fiber, value: Janet, sig: Signal) ---

        /* Shorthand for yielding to event loop in C */
        await :: proc() -> ! ---
        sleep_await :: proc(sec: f64) -> ! ---

        /* For use inside listeners - adds a timeout to the current fiber, such that
        * it will be resumed after sec seconds if no other event schedules the current fiber. */
        addtimeout :: proc(sec: f64) ---
        addtimeout_nil :: proc(sec: f64) ---
        ev_inc_refcount :: proc() ---
        ev_dec_refcount :: proc() ---

        /* Thread aware abstract types and helpers */
        abstract_begin_threaded :: proc(#by_ptr atype: AbstractType, size: uint) -> rawptr ---
        abstract_end_threaded :: proc(x: rawptr) -> rawptr ---
        abstract_threaded :: proc(#by_ptr atype: AbstractType, size: uint) -> rawptr ---
        abstract_incref :: proc(abst: rawptr) -> i32 ---
        abstract_decref :: proc(abst: rawptr) -> i32 ---

        /* Expose channel utilities */
        channel_make :: proc(limit: u32) -> ^Channel ---
        channel_make_threaded :: proc(limit: u32) -> ^Channel ---
        getchannel :: proc(argv: [^]Janet, n: i32) -> ^Channel ---
        optchannel :: proc(argv: [^]Janet, argc: i32, n: i32, dflt: ^Channel) -> Channel ---
        channel_give :: proc(channel: ^Channel, x: Janet) -> i32 ---
        channel_take :: proc(channel: ^Channel, out: ^Janet) -> i32 ---

        /* Expose some OS sync primitives */
        os_mutex_size :: proc() -> uint ---
        os_rwlock_size :: proc() -> uint ---
        os_mutex_init :: proc(mutex: ^OSMutex) ---
        os_mutex_deinit :: proc(mutex: ^OSMutex) ---
        os_mutex_lock :: proc(mutex: ^OSMutex) ---
        os_mutex_unlock :: proc(mutex: ^OSMutex) ---
        os_rwlock_init :: proc(rwlock: ^OSRWLock) ---
        os_rwlock_deinit :: proc(rwlock: ^OSRWLock) ---
        os_rwlock_rlock :: proc(rwlock: ^OSRWLock) ---
        os_rwlock_wlock :: proc(rwlock: ^OSRWLock) ---
        os_rwlock_runlock :: proc(rwlock: ^OSRWLock) ---
        os_rwlock_wunlock :: proc(rwlock: ^OSRWLock) ---

        /* Get last error from an IO operation */
        ev_lasterr :: proc() -> Janet ---


        /* API calls for quickly offloading some work in C to a new thread or thread pool. */
        ev_threaded_call :: proc(fp: ThreadedSubroutine, arguments: EVGenericMessage, cb: ThreadedCallback) ---
        ev_threaded_await :: proc(fp: ThreadedSubroutine, tag: i32, argi: i32, argp: rawptr) -> ! ---

        /* Post callback + userdata to an event loop. Takes the vm parameter to allow posting from other
        * threads or signal handlers. Use NULL to post to the current thread. */
        ev_post_event :: proc(vm: ^VM, cb: Callback, msg: EVGenericMessage) ---

        /* Callback used by janet_ev_threaded_await */
        ev_default_threaded_callback :: proc(return_value: EVGenericMessage) ---

        /* Read async from a stream */
        ev_read :: proc(stream: ^Stream, buf: ^Buffer, nbytes: i32) -> ! ---
        ev_readchunk :: proc(stream: ^Stream, buf: ^Buffer, nbytes: i32) -> ! ---


        ev_recv :: proc(stream: ^Stream, buf: ^Buffer, nbytes: i32, flags: i32)      -> ! ---
        ev_recvchunk :: proc(stream: ^Stream, buf: ^Buffer, nbytes: i32, flags: i32) -> ! ---
        ev_recvfrom :: proc(stream: ^Stream, buf: ^Buffer, nbytes: i32, flags: i32)  -> ! ---


        /* Write async to a stream */
        ev_write_buffer :: proc(stream: ^Stream, buf: ^Buffer)                            -> ! ---
        ev_write_string :: proc(stream: ^Stream, str: String)                             -> ! ---
        ev_send_buffer :: proc(stream: ^Stream, buf: ^Buffer, flags: i32)                 -> ! ---
        ev_send_string :: proc(stream: ^Stream, str: String, flags: i32)                  -> ! ---
        ev_sendto_buffer :: proc(stream: ^Stream, buf: ^Buffer, dest: rawptr, flags: i32) -> ! ---
        ev_sendto_string :: proc(stream: ^Stream, str: String, dest: rawptr, flags: i32)  -> ! ---

    }
} else {
    @(private)
    Janet_Ev :: struct {}
}

/* Assembly */
when ASSEMBLER {

    AssembleResult :: struct {
        funcdef: ^FuncDef,
        error: String,
        status: AssembleStatus
    }

    AssembleStatus :: enum {
        OK,
        ERROR
    }

    @(link_prefix="janet_")
    @(default_calling_convention="c")
    foreign janet {

        assem :: proc(source: Janet, flags: i32)   -> AssembleResult ---
        disasm :: proc(def: ^FuncDef)              -> Janet ---
        asm_decode_instruction :: proc(instr: u32) -> Janet ---
    }
}

when INT_TYPES {
    IntType :: enum {
        NONE,
        S64,
        U64
    }

    @(link_prefix="janet_")
    @(default_calling_convention="c")

    foreign janet {

        s64_type: AbstractType
        u64_type: AbstractType


        is_int :: proc(x: Janet) -> IntType ---
        wrap_s64 :: proc(x: i64) -> Janet ---
        wrap_u64 :: proc(x: u64) -> Janet ---
        unwrap_s64 :: proc(x: Janet) -> i64 ---
        unwrap_u64 :: proc(x: Janet) -> u64 ---
    }
}

when PRF {
    HASH_KEY_SIZE :: 16
}
