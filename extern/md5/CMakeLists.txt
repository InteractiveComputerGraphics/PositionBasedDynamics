add_library(MD5 STATIC
	md5.cpp
	md5.h
	)

set_target_properties(MD5 PROPERTIES FOLDER "External Dependencies")
