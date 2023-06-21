#include "sub.hpp"

using evec3 = Eigen::Vector3<halff>;
//using namespace half_float::literal;

attribRez Attrib(const closesthit& att, const las* ptlas) {
	attribRez rez;
	//法線を求める
	auto tri = ptlas->at(att.tri.blasId()).second->triangles.at(att.tri.triId());
	rez.norm = ((evec3(tri.at(1).data()) - evec3(tri.at(0).data())).cross(evec3(tri.at(2).data()) - evec3(tri.at(0).data()))).normalized();
	//ヒット点を求める
	rez.hitpoint = (evec3(att.r.way().data()) * att.vsTriRez.uvt().at(2)) + evec3(att.r.org().data());

	//鏡面反射
	auto a = (-evec3(att.r.way().data())).dot(rez.norm);
	evec3 refrectway = (2.0 * ((-evec3(att.r.way().data())).dot(rez.norm)) * rez.norm + evec3(att.r.way().data())).normalized();
	rez.refrect.way() = hvec3({ refrectway.x(),refrectway.y(),refrectway.z() });
	rez.refrect.org() = hvec3({ rez.hitpoint.x(),rez.hitpoint.y(),rez.hitpoint.z() });

	return rez;
}
attribRez Attrib(const closesthit& att, const tri<halff>& tri) {
	attribRez rez;
	//法線を求める
	rez.norm = ((evec3(tri.at(1).data()) - evec3(tri.at(0).data())).cross(evec3(tri.at(2).data()) - evec3(tri.at(0).data()))).normalized();
	//ヒット点を求める
	rez.hitpoint = (evec3(att.r.way().data()) * att.vsTriRez.uvt().at(2)) + evec3(att.r.org().data());

	//鏡面反射
	auto a = (-evec3(att.r.way().data())).dot(rez.norm);
	evec3 refrectway = (2.0 * ((-evec3(att.r.way().data())).dot(rez.norm)) * rez.norm + evec3(att.r.way().data())).normalized();
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


hmat4 MakeTranslate(const hvec3& v) {
	hmat4 ret;
	ret <<	1.0, 0.0, 0.0, v.x(),
			0.0, 1.0, 0.0, v.y(),
			0.0, 0.0, 1.0, v.z(),
			0.0, 0.0, 0.0, 1.0;
	return ret;
}
hmat4 MakeScale(const hvec3& v) {
	hmat4 ret;
	ret <<	v.x(), 0.0, 0.0, 0.0,
			0.0, v.y(), 0.0, 0.0,
			0.0, 0.0, v.z(), 0.0,
			0.0, 0.0, 0.0, 1.0;
	return ret;
}
hmat4 MakeScale(const halff& s) {
	return MakeScale(hvec3({ s,s,s }));
}
hmat4 ExtendMat(const hmat3& m) {
	hmat4 ret;
	for (int r = 0; r < 4; r++)
		for (int c = 0; c < 4; c++) {
			if (r < 3 && c < 3)
				ret(r, c) = m(r, c);
			else if (r == c)
				ret(r, c) = 1;
			else
				ret(r, c) = 0;
		}

	return ret;
}
hmat4 MakeEular(const hvec3& r) {
	auto rot = Eigen::AngleAxis<halff>(r.x(), Eigen::Vector3<halff>::UnitX())
		* Eigen::AngleAxis<halff>(r.y(), Eigen::Vector3<halff>::UnitY())
		* Eigen::AngleAxis<halff>(r.z(), Eigen::Vector3<halff>::UnitZ());

	return ExtendMat(rot.matrix());
}