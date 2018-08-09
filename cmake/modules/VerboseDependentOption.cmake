function(VERBOSE_DEPENDENT_OPTION _option_name)
    set(_options)
    set(_oneValueArgs OPTION DOC DEFAULT STATE_UNMET)
    set(_multiValueArgs DEPENDS_WARNING DEPENDS_OTHER)

    cmake_parse_arguments(_VDO "${_options}" "${_oneValueArgs}" "${_multiValueArgs}" ${ARGN})

    if(NOT DEFINED _VDO_OPTION)
        set(_VDO_OPTION ENABLE_${_option_name})
    endif()

    if(NOT DEFINED _VDO_DOC)
        set(_VDO_DOC "Enable/disable ${_option_name}")
    endif()

    if(NOT DEFINED _VDO_DEFAULT)
        set(_VDO_DEFAULT ON)
    endif()

    if(NOT DEFINED _VDO_STATE_UNMET)
        set(_VDO_STATE_UNMET OFF)
    endif()

    list(APPEND _all_deps ${_VDO_DEPENDS_WARNING} ${_VDO_DEPENDS_OTHER})

    if(_all_deps)
        list(REMOVE_DUPLICATES _all_deps)
    endif()

    unset(${_VDO_OPTION})
    unset(${_VDO_OPTION} PARENT_SCOPE)

    option(${_VDO_OPTION} "${_VDO_DOC}" "${_VDO_DEFAULT}")

    unset(_force)

    if(NOT DEFINED ${_VDO_OPTION}-NOTFIRSTRUN)
        set(${_VDO_OPTION} ${_VDO_DEFAULT} CACHE BOOL "${_VDO_DOC}" FORCE)
    endif()

    unset(_unmet_deps)

    foreach(_dep IN LISTS _all_deps)
        if(${_dep})
        else()
            list(APPEND _unmet_deps ${_dep})
        endif()
    endforeach()

    if(_unmet_deps)
        if(${_VDO_OPTION} AND NOT DEFINED ${_VDO_OPTION}-FORCED)
            unset(_verbose_dep_list)
            if(NOT DEFINED ${_VDO_OPTION}-NOTFIRSTRUN)
                foreach(_dep IN LISTS _unmet_deps)
                    if(_dep IN_LIST _VDO_DEPENDS_WARNING)
                        list(APPEND _verbose_dep_list ${_dep})
                    endif()
                endforeach()
            else()
                set(_verbose_dep_list ${_unmet_deps})
            endif()
            if(_verbose_dep_list)
                #string(REPLACE ";" "\n" _verbose_dep_list "${_verbose_dep_list}")
                message(WARNING "${_option_name} was set to OFF due to unmet conditions:\n${_verbose_dep_list}")
            endif()
        endif()
        if(NOT DEFINED ${_VDO_OPTION}-STORED)
            set(${_VDO_OPTION}-STORED "${${_VDO_OPTION}}" CACHE INTERNAL
                "Stored value of ${_VDO_OPTION} prior to forcing it OFF")
        endif()
        set(${_VDO_OPTION} OFF CACHE BOOL "${_VDO_DOC}" FORCE)
        set(${_VDO_OPTION}-FORCED "" CACHE INTERNAL "${_VDO_OPTION} was forced OFF")
    elseif(DEFINED ${_VDO_OPTION}-STORED AND DEFINED ${_VDO_OPTION}-FORCED)
        set(${_VDO_OPTION} "${${_VDO_OPTION}-STORED}" CACHE BOOL "${_VDO_DOC}" FORCE)
        unset(${_VDO_OPTION}-STORED CACHE)
        unset(${_VDO_OPTION}-FORCED CACHE)
    endif()

    set(${_VDO_OPTION}-NOTFIRSTRUN "" CACHE INTERNAL
        "Already processed ${_VDO_OPTION}, warnings will be generated for all unmet dependencies")

    set(${_VDO_OPTION}_ISSET "${${_VDO_OPTION}}" CACHE INTERNAL
        "Last value of ${_VDO_OPTION} for cmake_dependent_option()'s consumers")

    set_property(CACHE ${_VDO_OPTION} PROPERTY STRINGS "")
endfunction()
