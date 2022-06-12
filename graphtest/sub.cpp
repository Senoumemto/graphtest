#include "sub.hpp"

using evec3 = Eigen::Vector3<halff>;
using namespace half_float::literal;

attribRez Attrib(const closesthit& att, const sptr<tlas>& ptlas) {
	attribRez rez;
	//法線を求める
	auto tri = ptlas->at(att.tri.blasId()).second->triangles.at(att.tri.triId());
	rez.norm = ((evec3(tri.at(1).data()) - evec3(tri.at(0).data())).cross(evec3(tri.at(2).data()) - evec3(tri.at(0).data()))).normalized();
	//ヒット点を求める
	rez.hitpoint = (evec3(att.r.way().data()) * att.uvt.at(2)) + evec3(att.r.org().data());

	//鏡面反射
	auto a = (-evec3(att.r.way().data())).dot(rez.norm);
	evec3 refrectway = (2.0_h * ((-evec3(att.r.way().data())).dot(rez.norm)) * rez.norm + evec3(att.r.way().data())).normalized();
	rez.refrect.way() = hvec3({ refrectway.x(),refrectway.y(),refrectway.z() });
	rez.refrect.org() = hvec3({ rez.hitpoint.x(),rez.hitpoint.y(),rez.hitpoint.z() });

	return rez;
}

int MakeTestSquare(dmod& ret) {

	ret.resize(2);
	const dvec3 a({ -0.5,0.5,-1 }), b({ 0.5,0.5,-1 }), c({ 0.5,-0.5,-1 }), d({ -0.5,-0.5,-1 }), o({ 0,0,-1 });
	const dtri p({ a,b,o }), q({ b,c,o }), r({ c,d,o }), s({ d,a,o });

	ret = { p,q,r,s };

	return 0;
}

void ModLoader(const std::string& path, dmod& ret) {
	Assimp::Importer myImp;
	myImp.ReadFile(path, aiProcessPreset_TargetRealtime_MaxQuality);

	for (size_t m = 0; m < myImp.GetScene()->mNumMeshes; m++)
		for (size_t f = 0; f < myImp.GetScene()->mMeshes[m]->mNumFaces; f++) {
			auto p0 = myImp.GetScene()->mMeshes[m]->mVertices[myImp.GetScene()->mMeshes[m]->mFaces[f].mIndices[0]], p1 = myImp.GetScene()->mMeshes[m]->mVertices[myImp.GetScene()->mMeshes[m]->mFaces[f].mIndices[1]], p2 = myImp.GetScene()->mMeshes[m]->mVertices[myImp.GetScene()->mMeshes[m]->mFaces[f].mIndices[2]];
			ret.push_back(dtri({ dvec3({p0.x,p0.y,p0.z }),dvec3({p1.x,p1.y,p1.z }),dvec3({p2.x,p2.y,p2.z }) }));
		}


}

