import qbs
Module {
    Depends { name: "cpp" }
    property string odePath: "c:/msys64/home/kings/ode-0.16.2"
    cpp.includePaths: odePath + "/include"
    cpp.libraryPaths: odePath + "/ode/src/.libs"
    cpp.staticLibraries: "ode"
}

