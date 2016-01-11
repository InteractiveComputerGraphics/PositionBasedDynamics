#include "Shader.h"
#include <iostream>
#include <fstream>
#include <sstream>

using namespace PBD;

Shader::Shader(void)
{
	m_initialized = false;
	for (unsigned char i = 0; i < 3; i++)
		m_shaders[i] = 0;
	m_attributes.clear();
	m_uniforms.clear();
}

Shader::~Shader(void)
{
	m_initialized = false;
	m_attributes.clear();
	m_uniforms.clear();
	glDeleteProgram(m_program);
}

void Shader::compileShaderString(GLenum type, const std::string &source) 
{
	GLuint shader = glCreateShader (type);

	const GLchar *schr = source.c_str();
	glShaderSource(shader, 1, &schr, NULL);

	GLint status;
	glCompileShader(shader);
	glGetShaderiv (shader, GL_COMPILE_STATUS, &status);
	if (status == GL_TRUE)
	{
		if (type == GL_VERTEX_SHADER)
			m_shaders[0] = shader;
		else if (type == GL_GEOMETRY_SHADER)
			m_shaders[1] = shader;
		else if (type == GL_FRAGMENT_SHADER)
			m_shaders[2] = shader;
	}
	else
	{
		GLint infoLogLength;
		glGetShaderiv (shader, GL_INFO_LOG_LENGTH, &infoLogLength);
		GLchar *infoLog= new GLchar[infoLogLength];
		glGetShaderInfoLog (shader, infoLogLength, NULL, infoLog);
		std::cerr << "Compile log: " << infoLog << std::endl;
		delete [] infoLog;
	}
	
}


void Shader::createAndLinkProgram() 
{
	m_program = glCreateProgram ();
	for (unsigned char i = 0; i < 3; i++)
	{
		if (m_shaders[i] != 0)
			glAttachShader(m_program, m_shaders[i]);
	}

	GLint status;
	glLinkProgram (m_program);
	glGetProgramiv (m_program, GL_LINK_STATUS, &status);
	if (status == GL_TRUE)
		m_initialized = true;
	else
	{
		GLint infoLogLength;

		glGetProgramiv (m_program, GL_INFO_LOG_LENGTH, &infoLogLength);
		GLchar *infoLog= new GLchar[infoLogLength];
		glGetProgramInfoLog (m_program, infoLogLength, NULL, infoLog);
		std::cerr << "Link log: " << infoLog << std::endl;
		delete [] infoLog;
	}

	for (unsigned char i = 0; i < 3; i++)
		glDeleteShader(m_shaders[i]);
}

void Shader::begin() 
{
	if (m_initialized)
		glUseProgram(m_program);
}

void Shader::end() 
{
	if (m_initialized)
		glUseProgram(0);
}

void Shader::addAttribute(const std::string &attribute)
{
	m_attributes[attribute]= glGetAttribLocation(m_program, attribute.c_str());
}

GLuint Shader::getAttribute(const std::string &attribute)
{
	return m_attributes[attribute];
}

void Shader::addUniform(const std::string &uniform)
{
	m_uniforms[uniform] = glGetUniformLocation(m_program, uniform.c_str());
}

GLuint Shader::getUniform(const std::string &uniform)
{
	return m_uniforms[uniform];
}

void Shader::compileShaderFile(GLenum whichShader, const std::string &filename)
{
	std::ifstream fp;
	fp.open(filename.c_str(), std::ios_base::in);
	if(fp) 
	{
		std::ostringstream output;
		output << fp.rdbuf();
		fp.close();

		compileShaderString(whichShader, output.str());
	} 
	else 
	{
		std::cerr << "Error occurred while loading shader: " << filename << std::endl;
	}
}

bool Shader::isInitialized()
{
	return m_initialized;
}
