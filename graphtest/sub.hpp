#pragma once

#include"general.hpp"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags
#include <bmp.h>

using dmod = mod<double>;
using dtri = tri<double>;
using dvec3 = vec3<double>;

//テストで1x1の正方形をｚ＝-１に作る
int MakeTestSquare(dmod& ret) {

	ret.resize(2);
	const dvec3 a({ -0.5,0.5,-1 }), b({ 0.5,0.5,-1 }), c({ 0.5,-0.5,-1 }), d({ -0.5,-0.5,-1 }), o({0,0,-1});
	const dtri p({ a,b,o }), q({ b,c,o }), r({ c,d,o }), s({d,a,o});

	ret = { p,q,r,s };

	return 0;
}

void ModLoader(const std::string& path,dmod& ret) {
	Assimp::Importer myImp;
	myImp.ReadFile(path,aiProcessPreset_TargetRealtime_MaxQuality);

	for (int m = 0; m < myImp.GetScene()->mNumMeshes; m++)
		for (int f = 0; f < myImp.GetScene()->mMeshes[m]->mNumFaces; f++) {
			auto p0 = myImp.GetScene()->mMeshes[m]->mVertices[myImp.GetScene()->mMeshes[m]->mFaces[f].mIndices[0]],p1 = myImp.GetScene()->mMeshes[m]->mVertices[myImp.GetScene()->mMeshes[m]->mFaces[f].mIndices[1]], p2 = myImp.GetScene()->mMeshes[m]->mVertices[myImp.GetScene()->mMeshes[m]->mFaces[f].mIndices[2]];
			ret.push_back(dtri({dvec3({p0.x,p0.y,p0.z }),dvec3({p1.x,p1.y,p1.z }),dvec3({p2.x,p2.y,p2.z }) }));
		}


}

template<size_t res>void PrintBmp(const std::string& path,const bitmap<res>& dat) {
	sptr<img> g(new img);
	g->width = res;
	g->height = res;

	for (int y = 0; y < g->height; y++)
		for (int x = 0; x < g->width; x++) {
			g->data[y][x].r = dat.at(y * res + x).at(0)*255;
			g->data[y][x].g = dat.at(y * res + x).at(1)*255;
			g->data[y][x].b = dat.at(y * res + x).at(2)*255;
		}

	WriteBmp(path.c_str(), g.get());
}