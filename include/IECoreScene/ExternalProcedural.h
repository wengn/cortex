//////////////////////////////////////////////////////////////////////////
//
//  Copyright (c) 2014, Image Engine Design Inc. All rights reserved.
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

#ifndef IECORESCENE_EXTERNALPROCEDURAL_H
#define IECORESCENE_EXTERNALPROCEDURAL_H

#include "IECoreScene/Export.h"
#include "IECoreScene/VisibleRenderable.h"

namespace IECoreScene
{

/// \ingroup renderingGroup
/// \ingroup coreGroup
class IECORESCENE_API ExternalProcedural : public VisibleRenderable
{

	public :

		ExternalProcedural( const std::string &fileName = "", const Imath::Box3f &bound = Imath::Box3f(), const IECore::CompoundData *parameters = nullptr );
		~ExternalProcedural() override;

		IE_CORE_DECLAREEXTENSIONOBJECT( ExternalProcedural, ExternalProceduralTypeId, VisibleRenderable );

		// \todo : Rename to just "name"?
		// In renderers like Arnold, procedurals don't load files directly, instead procedural files
		// declare new node type names, and you create a procedural using this name.
		void setFileName( const std::string &fileName );
		const std::string &getFileName() const;

		void setBound( const Imath::Box3f &bound );
		const Imath::Box3f &getBound() const;

		IECore::CompoundData *parameters();
		const IECore::CompoundData *parameters() const;

		void render( Renderer *renderer ) const override;
		Imath::Box3f bound() const override;

    void readMeshPoints();
    std::vector<Imath::Box3f> getMeshBounds() const;
    std::vector<std::vector<Imath::V3f>> getMeshPoints() const;
    std::vector<std::vector<int>> getIndices() const;
    std::vector<std::vector<int>> getVertCount() const;

	private :

		std::string m_fileName;
		Imath::Box3f m_bound;
		IECore::CompoundDataPtr m_parameters;

    std::vector<Imath::Box3f> m_meshBounds; //Naiqi's change
    std::vector<std::vector<Imath::V3f>> m_meshes; //Naiqi's change
    std::vector<std::vector<int>> m_vertIndices; //Naiqi's change
    std::vector<std::vector<int>> m_vertCounts; //Number of vertex per face
};

IE_CORE_DECLAREPTR( ExternalProcedural );

} // namespace IECoreScene

#endif // IECORESCENE_EXTERNALPROCEDURAL_H
