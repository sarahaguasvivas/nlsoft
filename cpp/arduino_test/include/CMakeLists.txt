file (GLOB HEADERS include/*)
foreach( testsourcefile get_filename_component(NAME ${HEADERS} ABSOLUTE)
    string( REPLACE ".h" "" testname ${testsourcefile} )
    string( REPLACE ".h" "" testname ${testsourcefile} )

    add_library( ${testname} ${testsourcefile} ))
endforeach( testsourcefile ${HEADERS} )
