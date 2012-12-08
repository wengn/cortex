//////////////////////////////////////////////////////////////////////////
//
//  Copyright (c) 2007-2012, Image Engine Design Inc. All rights reserved.
//
//  Copyright 2010 Dr D Studios Pty Limited (ACN 127 184 954) (Dr. D Studios), 
//  its affiliates and/or its licensors.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are
//  met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of Image Engine Design nor the names of any
//       other contributors to this software may be used to endorse or
//       promote products derived from this software without specific prior
//       written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
//  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
//  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
//  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
//  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
//  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
//  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
//  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//////////////////////////////////////////////////////////////////////////

#ifndef IECOREGL_SHADER_H
#define IECOREGL_SHADER_H

#include <boost/utility.hpp>

#include "IECoreGL/GL.h"
#include "IECoreGL/Bindable.h"
#include "IECoreGL/TypeIds.h"

#include "IECore/CompoundData.h"

namespace IECoreGL
{

IE_CORE_FORWARDDECLARE( Shader );
IE_CORE_FORWARDDECLARE( Texture );

/// A class to represent GLSL shaders.
class Shader : public IECore::RunTimeTyped
{

	public :

		IE_CORE_DECLARERUNTIMETYPEDEXTENSION( IECoreGL::Shader, ShaderTypeId, IECore::RunTimeTyped );

		/// Either vertexSource or fragmentSource may be empty to use a simple default
		/// shader for that shader component. Throws an Exception if the shader fails
		/// to compile, or if the OpenGL version isn't sufficient to support shaders.
		Shader( const std::string &vertexSource, const std::string &fragmentSource );
		virtual ~Shader();

		/// Fills the passed vector with the names of all uniform shader parameters.
		/// Structures will use the struct.component convention used in GLSL.
		/// Arrays will be returned as a single name, rather than the list array[0],
		/// array[n] names used internally in OpenGL.
		void uniformParameterNames( std::vector<std::string> &names ) const;
		GLint uniformParameter( const std::string &name, GLenum &type, GLint &size, size_t &textureUnit ) const; 
				
		/// Fills the passed vector with the names of all vertex shader parameters.
		void vertexParameterNames( std::vector<std::string> &names ) const;		
		GLint vertexAttribute( const std::string &name, GLenum &type, GLint &size ) const; 
		
		/// Shaders are only useful when associated with a set of values for
		/// their uniform parameters and vertex attributes, and to render
		/// different objects in different forms different sets of values
		/// will be a necessary. The Setup class encapsulates a set of such
		/// values and provides a means of cleanly binding and unbinding the
		/// Shader using them.
		class Setup : public IECore::RefCounted
		{
		
			public :
		
				Setup( ConstShaderPtr shader );
				
				const Shader *shader() const;
		
				void addUniformParameter( const std::string &name, ConstTexturePtr value );
				void addUniformParameter( const std::string &name, IECore::ConstDataPtr value );
				void addVertexAttribute( const std::string &name, IECore::ConstDataPtr value );
		
				/// The ScopedBinding class cleanly binds and unbinds the shader
				/// Setup, making the shader current and setting the uniform
				/// and vertex values as necessary.
				class ScopedBinding
				{
				
					public :
					
						/// Binds the setup. It is the responsibility of the
						/// caller to ensure the setup remains alive for
						/// the lifetime of the ScopedBinding.
						ScopedBinding( const Setup &setup );
						/// Unbinds the setup, reverting to the previous state.
						~ScopedBinding();
				
					private :
					
						GLint m_previousProgram;
						const Setup &m_setup;
				
				};
		
			private :
		
				IE_CORE_FORWARDDECLARE( MemberData );
				
				MemberDataPtr m_memberData;
					
		};
		
		IE_CORE_DECLAREPTR( Setup );
		
		//! @name Default shader source.
		/// These functions return the default shader source used
		/// when source isn't provided to the constructor.
		///////////////////////////////////////////////////////////
		//@{
		static const std::string &defaultVertexSource();
		static const std::string &defaultFragmentSource();
		//@}
		
		//! @name Built in shaders
		/// These functions provide access to static instances of
		/// various simple but useful shaders.
		///////////////////////////////////////////////////////////
		//@{
		/// Returns a shader which shades as a constant flat color.
		static ShaderPtr constant();
		/// Returns a shader which shades as a facing ratio.
		static ShaderPtr facingRatio();
		//@}

	private :

		IE_CORE_FORWARDDECLARE( Implementation )
		ImplementationPtr m_implementation;

};

} // namespace IECoreGL

#endif // IECOREGL_SHADER_H
