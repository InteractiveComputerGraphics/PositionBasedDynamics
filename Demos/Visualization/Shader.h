#ifndef __Shader_h__
#define __Shader_h__

#include <map>
#include <string>
#include <glad/gl.h>

namespace PBD
{
	class Shader
	{
	public:
		Shader();
		~Shader();

		void compileShaderString(GLenum whichShader, const std::string &source);
		void compileShaderFile(GLenum whichShader, const std::string &filename);
		void createAndLinkProgram();
		void destroy();
		void addAttribute(const std::string &attribute);
		void addUniform(const std::string &uniform);
		bool isInitialized();

		void begin();
		void end();

		//An indexer that returns the location of the attribute/uniform
		GLuint getAttribute(const std::string &attribute);
		GLuint getUniform(const std::string &uniform);

	private:
		bool m_initialized;
		GLuint	m_program;
		std::map<std::string, GLuint> m_attributes;
		std::map<std::string, GLuint> m_uniforms;
		GLuint m_shaders[3];	
	};
}

#endif
