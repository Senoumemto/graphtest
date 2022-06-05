#include "general.hpp"

using evec3 = Eigen::Vector3<halff>;

const extern sindex RAYNUM_LIMIT_BRUNCH;
const extern exindex RAYNUM_LIMIT_ALL;

using brunch = toolkit::materialer<RAYNUM_LIMIT_ALL, RAYNUM_LIMIT_BRUNCH>::brunch;

struct attribRez {
	evec3 norm;
	evec3 hitpoint;
	ray refrect;
};
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

payloadContent HitMirror(const closesthit& att, brunch& nextgenlocal, sptr<tlas> ptlas,bool& isTerminate) {
	using namespace half_float::literal;
	using evec3 = Eigen::Vector3<halff>;

	auto attrib = Attrib(att, ptlas);

	//反射光を登録
	nextgenlocal.push_head(parentedRay({ att.r.index(), attrib.refrect }));

	//反射するのでtermでない
	isTerminate = false;
	return payloadContent({ 0.5_h,0.5_h,0.5_h });
}
payloadContent HitLight(const closesthit& att, brunch& nextgenlocal, sptr<tlas> ptlas, bool& isTerminate) {
	using namespace half_float::literal;
	using evec3 = Eigen::Vector3<halff>;

	auto attrib = Attrib(att, ptlas);
	nextgenlocal.push_head(parentedRay({ att.r.index(), attrib.refrect }));

	//反射するのでtermでない
	isTerminate = false;
	return payloadContent({1.0_h,1.0_h,1.0_h}, { std::abs<halff>(attrib.norm.x()),std::abs<halff>(attrib.norm.y()),std::abs<halff>(attrib.norm.z()) });
}

payloadContent MissShader(const closesthit& str, brunch& nextgenlocal, sptr<tlas> ptlas,bool& isTerminate) {
	using evec3 = Eigen::Vector3<halff>;
	using namespace half_float::literal;

	evec3 direction(str.r.way().data());
	evec3 light(0.0_h, -1.0_h, 0.0_h);

	halff doter = direction.normalized().dot(-light.normalized());
	doter = std::max(0.0_h, doter);
	doter = pow(doter, 1.0_h);

	//無限遠に飛んで行った
	isTerminate = true;

	doter = 0.0_h;
	halff amb = +0.05_h;//環境光
	return payloadContent({ 1.0_h,1.0_h ,1.0_h }, { doter + amb,doter + amb,doter + amb });
}

payloadContent HitPos(const closesthit& att, brunch& nextgenlocal, sptr<tlas> ptlas, bool& isTerminate) {
	using namespace half_float::literal;
	using evec3 = Eigen::Vector3<halff>;

	//法線を求める
	auto tri = ptlas->at(att.tri.blasId()).second->triangles.at(att.tri.triId());
	evec3 norm = ((evec3(tri.at(1).data()) - evec3(tri.at(0).data())).cross(evec3(tri.at(2).data()) - evec3(tri.at(0).data()))).normalized();
	//ヒット点を求める
	evec3 hitpoint = (evec3(att.r.way().data()) * att.uvt.at(2)) + evec3(att.r.org().data());

	//鏡面反射
	auto a = (-evec3(att.r.way().data())).dot(norm);
	evec3 refrectway = (norm * a * 2.0_h - evec3(att.r.way().data())).normalized();// evec3(att.r.way().data()) + norm * (a * -2.0_h);


	ray ref;
	ref.way() = hvec3({ refrectway.x(),refrectway.y(),refrectway.z() });
	ref.org() = hvec3({ hitpoint.x(),hitpoint.y(),hitpoint.z() });
	nextgenlocal.push_head(parentedRay({ att.r.index(), ref }));

	
	return payloadContent({ 1.0_h,1.0_h ,1.0_h }, { 0.0_h,hitpoint.y() / 20.0_h ,-hitpoint.z() / 20.0_h });
	//return payloadContent({1.0_h,1.0_h,1.0_h}, { std::abs<halff>(norm.x()),std::abs<halff>(norm.y()),std::abs<halff>(norm.z()) });
}

payloadContent HitWay(const closesthit& att, brunch& nextgenlocal, sptr<tlas> ptlas, bool& isTerminate) {
	using namespace half_float::literal;
	using evec3 = Eigen::Vector3<halff>;

	//法線を求める
	auto tri = ptlas->at(att.tri.blasId()).second->triangles.at(att.tri.triId());
	evec3 norm = ((evec3(tri.at(1).data()) - evec3(tri.at(0).data())).cross(evec3(tri.at(2).data()) - evec3(tri.at(0).data()))).normalized();
	//ヒット点を求める
	evec3 hitpoint = (evec3(att.r.way().data()) * att.uvt.at(2)) + evec3(att.r.org().data());

	//鏡面反射
	auto a = (-evec3(att.r.way().data())).dot(norm);
	//evec3 refrectway = (norm * a * 2.0_h - evec3(att.r.way().data())).normalized();// evec3(att.r.way().data()) + norm * (a * -2.0_h);
	evec3 refrectway = (2.0_h * ((-evec3(att.r.way().data())).dot(norm)) * norm + evec3(att.r.way().data())).normalized();


	ray ref;
	ref.way() = hvec3({ refrectway.x(),refrectway.y(),refrectway.z() });
	ref.org() = hvec3({ hitpoint.x(),hitpoint.y(),hitpoint.z() });

	isTerminate = true;
	//return payloadContent({ 1.0_h,1.0_h ,1.0_h }, { 1.0_h,1.0_h,1.0_h });
	return payloadContent({ 1.0_h,1.0_h ,1.0_h }, { ref.way().x() ,ref.way().y() ,ref.way().z() });
	//return payloadContent({1.0_h,1.0_h,1.0_h}, { std::abs<halff>(norm.x()),std::abs<halff>(norm.y()),std::abs<halff>(norm.z()) });
}