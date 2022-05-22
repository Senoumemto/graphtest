#include "general.hpp"

using evec3 = Eigen::Vector3<halff>;

const extern sindex RAYNUM_LIMIT_BRUNCH;
const extern exindex RAYNUM_LIMIT_ALL;

using brunch = toolkit::materialer<RAYNUM_LIMIT_ALL, RAYNUM_LIMIT_BRUNCH>::brunch;

payloadContent HitShader(const closesthit& att, brunch& nextgenlocal, sptr<tlas> ptlas,bool& isTerminate) {
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

	//std::cout << norm.norm().operator float() << std::endl;
	if (norm.z() <= 0.0001_h)
		int a = 0;

	//反射するのでtermでない
	isTerminate = false;
	//return payloadContent({ 0.5_h,0.5_h,0.5_h });
	//return payloadContent({ 1.0_h,1.0_h ,1.0_h }, { std::abs<halff>(refrectway.x()), std::abs<halff>(refrectway.y()), std::abs<halff>(refrectway.z()) });
	return payloadContent({std::abs<halff>(norm.x()),std::abs<halff>(norm.y()),std::abs<halff>(norm.z())});
	//return payloadContent({ std::abs<halff>(refrectway.x()),std::abs<halff>(refrectway.y()),std::abs<halff>(refrectway.z()) });
}
payloadContent MissShader(const closesthit& str, brunch& nextgenlocal, sptr<tlas> ptlas,bool& isTerminate) {
	using evec3 = Eigen::Vector3<halff>;
	using namespace half_float::literal;

	evec3 direction(str.r.way().data());
	evec3 light(1.0_h, -1.0_h, -0.2_h);

	halff doter = direction.normalized().dot(-light.normalized());
	doter = std::max(0.0_h, doter);
	doter = pow(doter, 1.0_h);

	//無限遠に飛んで行った
	isTerminate = true;

	doter = 0;
	halff amb = +1.0_h;//環境光
	return payloadContent({ 1.0_h,1.0_h ,1.0_h }, { doter + amb,doter + amb,doter + amb });
}