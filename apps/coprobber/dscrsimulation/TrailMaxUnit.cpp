#include "TrailMaxUnit.h"
#include "GLUtil.h"

template<>
tDirection TrailMaxUnit<xyLoc,tDirection,MapEnvironment>::nomove() {
	return kStay;
};

template<>
tDirection TrailMaxUnit<xyLoc,tDirection,AbsMapEnvironment>::nomove() {
	return kStay;
};

template<>
graphMove TrailMaxUnit<graphState,graphMove,GraphEnvironment>::nomove() {
	return graphMove(current_pos,current_pos);
};

template<>
graphMove TrailMaxUnit<graphState,graphMove,AbstractionGraphEnvironment>::nomove() {
	return graphMove(current_pos,current_pos);
};


template<>
void TrailMaxUnit<xyLoc,tDirection,MapEnvironment>::OpenGLDraw( int, MapEnvironment *env, SimulationInfo<xyLoc,tDirection,MapEnvironment>* ) {
	GLdouble xx, yy, zz, rad;
	Map *m = env->GetMap();
	if( current_pos.x >= m->getMapWidth() || current_pos.y >= m->getMapHeight() ) {
		fprintf( stderr, "Warning: TrailMaxUnit is out of bounds. Could not draw it.\n" );
		return;
	}
	m->getOpenGLCoord( current_pos.x, current_pos.y, xx, yy, zz, rad );
	if( done )
		glColor3d( 0., 0., 0. ); // turn black when done
	else
		glColor3f( r, g, b );
	DrawSphere( xx, yy, zz, rad );

	// draw the path in front of us
	if( pathcache.size() > 0 && !done) {
		glBegin(GL_LINE_STRIP);
		glVertex3f( xx, yy, zz-rad/2 );
		for( unsigned int i = 0; i < pathcache.size(); i++ ) {
			m->getOpenGLCoord( pathcache[i].x, pathcache[i].y, xx, yy, zz, rad );
			glVertex3f( xx, yy, zz-rad/2 );
		}
		glEnd();
	}

	return;
};

template<>
void TrailMaxUnit<xyLoc,tDirection,AbsMapEnvironment>::OpenGLDraw( int, AbsMapEnvironment *env, SimulationInfo<xyLoc,tDirection,AbsMapEnvironment>* ) {
	GLdouble xx, yy, zz, rad;
	Map *m = env->GetMap();
	if( current_pos.x >= m->getMapWidth() || current_pos.y >= m->getMapHeight() ) {
		fprintf( stderr, "Warning: TrailMaxUnit is out of bounds. Could not draw it.\n" );
		return;
	}
	m->getOpenGLCoord( current_pos.x, current_pos.y, xx, yy, zz, rad );
	if( done )
		glColor3d( 0., 0., 0. ); // turn black when done
	else
		glColor3f( r, g, b );
	DrawSphere( xx, yy, zz, rad );

	// draw the path in front of us
	if( pathcache.size() > 0 && !done) {
		glBegin(GL_LINE_STRIP);
		glVertex3f( xx, yy, zz-rad/2 );
		for( unsigned int i = 0; i < pathcache.size(); i++ ) {
			m->getOpenGLCoord( pathcache[i].x, pathcache[i].y, xx, yy, zz, rad );
			glVertex3f( xx, yy, zz-rad/2 );
		}
		glEnd();
	}

	return;
};

template<>
void TrailMaxUnit<graphState,graphMove,GraphEnvironment>::OpenGLDraw( int, GraphEnvironment *env, SimulationInfo<graphState,graphMove,GraphEnvironment>* ) {
	node *n  = env->GetGraph()->GetNode( current_pos );
	GLdouble x, y, z, rad = 0.005;
	x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
	y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
	z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
	if( done )
		glColor3d( 0., 0., 0. ); // turn black when done
	else
		glColor3f( r, g, b );
	DrawSphere( x, y, z, rad );

	// draw the path in front of us
	if( pathcache.size() > 0 && !done) {
		glBegin(GL_LINE_STRIP);
		glVertex3f( x, y, z-rad/2 );
		for( unsigned int i = 0; i < pathcache.size(); i++ ) {
			n  = env->GetGraph()->GetNode( pathcache[i] );
			x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
			y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
			z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
			glVertex3f( x, y, z-rad/2 );
		}
		glEnd();
	}

	return;

};

template<>
void TrailMaxUnit<graphState,graphMove,AbstractionGraphEnvironment>::OpenGLDraw( int, AbstractionGraphEnvironment *env, SimulationInfo<graphState,graphMove,AbstractionGraphEnvironment>* ) {
	node *n  = env->GetGraph()->GetNode( current_pos );
	GLdouble x, y, z, rad = 0.005;
	x = n->GetLabelF(GraphAbstractionConstants::kXCoordinate);
	y = n->GetLabelF(GraphAbstractionConstants::kYCoordinate);
	z = n->GetLabelF(GraphAbstractionConstants::kZCoordinate);
	if( done )
		glColor3d( 0., 0., 0. ); // turn black when done
	else
		glColor3f( r, g, b );
	DrawSphere( x, y, z, rad );

	// draw the path in front of us
	if( pathcache.size() > 0 && !done) {
		glBegin(GL_LINE_STRIP);
		glVertex3f( x, y, z-rad/2 );
		for( unsigned int i = 0; i < pathcache.size(); i++ ) {
			n  = env->GetGraph()->GetNode( pathcache[i] );
			x = n->GetLabelF(GraphAbstractionConstants::kXCoordinate);
			y = n->GetLabelF(GraphAbstractionConstants::kYCoordinate);
			z = n->GetLabelF(GraphAbstractionConstants::kZCoordinate);
			glVertex3f( x, y, z-rad/2 );
		}
		glEnd();
	}

	return;

};