#include "general.hpp"

payload HitShader(const closesthit& att, parentedRays& nextgen, exindicesWithHead* terminates, sptr<tlas> ptlas) {
	using namespace half_float::literal;
	using evec3 = Eigen::Vector3<halff>;

	//法線を求める
	auto tri = ptlas->at(att.tri.blasId()).second->triangles.at(att.tri.triId());
	evec3 norm = (evec3(tri.at(1).data()) - evec3(tri.at(0).data())).cross(evec3(tri.at(2).data()) - evec3(tri.at(0).data())).normalized();
	//ヒット点を求める
	evec3 hitpoint = (evec3(att.r.way().data()) * att.uvt.at(2)) + evec3(att.r.org().data());

	//鏡面反射
	auto a = (-evec3(att.r.way().data())).dot(norm);
	evec3 refrectway = (norm * a * 2.0_h - evec3(att.r.way().data())).normalized();// evec3(att.r.way().data()) + norm * (a * -2.0_h);


	ray ref;
	ref.way() = hvec3({ refrectway.x(),refrectway.y(),refrectway.z() });
	ref.org() = hvec3({ hitpoint.x(),hitpoint.y(),hitpoint.z() });
	nextgen.push_back(parentedRay({ std::numeric_limits<exindex>::max(), ref }));

	//return hvec3({ 1.0_h,1.0_h,1.0_h });
	return { hvec3({ std::abs<halff>(refrectway.x()),std::abs<halff>(refrectway.y()) ,std::abs<halff>(refrectway.z()) }) ,std::numeric_limits<exindex>::max()};
}
payload MissShader(const closesthit& str, parentedRays& nextgen, exindicesWithHead* terminates, sptr<tlas> ptlas) {
	using evec3 = Eigen::Vector3<halff>;
	using namespace half_float::literal;

	evec3 direction(str.r.way().data());
	evec3 light(0.0_h, -1.0_h, 0.0_h);

	auto doter = direction.dot(-light);
	doter = std::max(0.0_h, doter);


	terminates->push_head(str.r.index());
	return { hvec3({ doter,doter,doter }), std::numeric_limits<exindex>::max() };
}