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

#include "IECoreScene/ExternalProcedural.h"

#include "IECoreScene/Renderer.h"

#include "IECoreArnold/NodeAlgo.h" //Naiqi's change
#include "IECoreArnold/ParameterAlgo.h" //Naiqi's change
#include "ai.h" //Naiqi's change
#include "ai_array.h"
#include "IECore/SimpleTypedData.h"
#include "IECoreArnold/ParameterAlgo.h"

using namespace IECore;
using namespace IECoreScene;
using namespace IECoreArnold; //Naiqi's change

IE_CORE_DEFINEOBJECTTYPEDESCRIPTION( ExternalProcedural );

static const IndexedIO::EntryID g_fileNameEntry( "fileName" );
static const IndexedIO::EntryID g_boundEntry( "bound" );
static const IndexedIO::EntryID g_parametersEntry( "parameters" );

static const unsigned int g_ioVersion = 0;

ExternalProcedural::ExternalProcedural( const std::string &fileName, const Imath::Box3f &bound, const CompoundData *parameters )
	:	m_fileName( fileName ), m_bound( bound ), m_parameters( parameters ? parameters->copy() : new CompoundData )
{
}

ExternalProcedural::~ExternalProcedural()
{
}

void ExternalProcedural::setFileName( const std::string &fileName )
{
	m_fileName = fileName;
}

const std::string &ExternalProcedural::getFileName() const
{
	return m_fileName;
}

void ExternalProcedural::setBound( const Imath::Box3f &bound )
{
	m_bound = bound;
}

const Imath::Box3f &ExternalProcedural::getBound() const
{
	return m_bound;
}

CompoundData *ExternalProcedural::parameters()
{
	return m_parameters.get();
}

const CompoundData *ExternalProcedural::parameters() const
{
	return m_parameters.get();
}

void ExternalProcedural::render( Renderer *renderer ) const
{
	renderer->procedural( new Renderer::ExternalProcedural( m_fileName, m_bound, m_parameters->readable() ) );
}

Imath::Box3f ExternalProcedural::bound() const
{
	return m_bound;
}

void ExternalProcedural::copyFrom( const Object *other, CopyContext *context )
{
	VisibleRenderable::copyFrom( other, context );
	const ExternalProcedural *tOther = static_cast<const ExternalProcedural *>( other );
	m_fileName = tOther->m_fileName;
	m_bound = tOther->m_bound;
	m_parameters = tOther->m_parameters->copy();
}

void ExternalProcedural::save( SaveContext *context ) const
{
	VisibleRenderable::save( context );
	IndexedIOPtr container = context->container( staticTypeName(), g_ioVersion );
	container->write( g_fileNameEntry, m_fileName );
	container->write( g_boundEntry, m_bound.min.getValue(), 6 );
	context->save( m_parameters.get(), container.get(), g_parametersEntry );
}

void ExternalProcedural::load( LoadContextPtr context )
{
	VisibleRenderable::load( context );
	unsigned int v = g_ioVersion;
	ConstIndexedIOPtr container = context->container( staticTypeName(), v );
	container->read( g_fileNameEntry, m_fileName );

	float *b = m_bound.min.getValue();
	container->read( g_boundEntry, b, 6 );

	m_parameters = context->load<CompoundData>( container.get(), g_parametersEntry );

}

bool ExternalProcedural::isEqualTo( const Object *other ) const
{
	if( !VisibleRenderable::isEqualTo( other ) )
	{
		return false;
	}

	const ExternalProcedural *tOther = static_cast<const ExternalProcedural *>( other );

	return
		m_fileName == tOther->m_fileName &&
		m_bound == tOther->m_bound &&
		m_parameters->isEqualTo( tOther->m_parameters.get() );
}

void ExternalProcedural::memoryUsage( Object::MemoryAccumulator &a ) const
{
	VisibleRenderable::memoryUsage( a );
	a.accumulate( m_fileName.capacity() );
	a.accumulate( sizeof( m_bound ) );
	a.accumulate( m_parameters.get() );
}

void ExternalProcedural::hash( MurmurHash &h ) const
{
	VisibleRenderable::hash( h );
	h.append( m_fileName );
	h.append( m_bound );
	m_parameters->hash( h );
}

//Naiqi's change
//Parse the ass file and parse polymesh info
void ExternalProcedural::readMeshPoints()
{
  // Current implementation: the ass file path is not stored on the m_fileName, you have to get the information this way
  const StringData* filePath = parameters()->member< StringData >(InternedString("filename"));
  if(!filePath)
    return;

  if(AiASSLoad(filePath->readable().c_str(), AI_NODE_SHAPE) != -1)
  {
    AtNodeIterator * iter = AiUniverseGetNodeIterator(AI_NODE_SHAPE);
    while(!AiNodeIteratorFinished(iter))
    {
      AtNode * node = AiNodeIteratorGetNext(iter);
      if(!node)
        continue;

      // Currently assuming matrix are all identity and all transformation are recorded in vertex position
      uint32_t numElem = AiArrayGetNumElements(AiNodeGetArray(node, AtString("vlist")));
      std::vector<AtVector> points;
      if(numElem)
      {
        points.assign(static_cast<size_t>(numElem), AtVector());
        for(size_t i = 0; i < static_cast<size_t>(numElem); ++i)
        {
          points[i] = AiArrayGetVec(AiNodeGetArray(node, AtString("vlist")), i);
        }

        float minX{ points[0].x}, minY{ points[0].y}, minZ{ points[0].z};
        float maxX{ minX }, maxY{ minY }, maxZ{ minZ };
        for(auto i: points)
        {
          minX = i.x < minX ? i.x : minX;
          minY = i.y < minY ? i.y : minY;
          minZ = i.z < minZ ? i.z : minZ;
          maxX = i.x > maxX ? i.x : maxX;
          maxY = i.y > maxY ? i.y : maxY;
          maxZ = i.z > maxZ ? i.z : maxZ;
        }
        m_meshBounds.emplace_back(Imath::V3f(minX, minY,minZ), Imath::V3f(maxX, maxY, maxZ));
      }
    }
    AiNodeIteratorDestroy(iter);
  }
}

std::vector<Imath::Box3f> ExternalProcedural::getMeshBounds() const
{
  return m_meshBounds;
}

