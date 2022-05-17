#include "general.hpp"

using evec3 = Eigen::Vector3<halff>;

payloadContent HitShader(const closesthit& att, parentedRays& nextgen, exindicesWithHead* terminates, sptr<tlas> ptlas) {
	using namespace half_float::literal;
	using evec3 = Eigen::Vector3<halff>;

	//�@�������߂�
	auto tri = ptlas->at(att.tri.blasId()).second->triangles.at(att.tri.triId());
	evec3 norm = ((evec3(tri.at(1).data()) - evec3(tri.at(0).data())).cross(evec3(tri.at(2).data()) - evec3(tri.at(0).data()))).normalized();
	//�q�b�g�_�����߂�
	evec3 hitpoint = (evec3(att.r.way().data()) * att.uvt.at(2)) + evec3(att.r.org().data());

	//���ʔ���
	auto a = (-evec3(att.r.way().data())).dot(norm);
	evec3 refrectway = (norm * a * 2.0_h - evec3(att.r.way().data())).normalized();// evec3(att.r.way().data()) + norm * (a * -2.0_h);


	ray ref;
	ref.way() = hvec3({ refrectway.x(),refrectway.y(),refrectway.z() });
	ref.org() = hvec3({ hitpoint.x(),hitpoint.y(),hitpoint.z() });
	nextgen.push_back(parentedRay({ att.r.index(), ref }));

	return payloadContent({ 0.7_h,0.5_h, 0.5_h,});
	//return payloadContent({ std::abs<halff>(refrectway.x()),std::abs<halff>(refrectway.y()),std::abs<halff>(refrectway.z()) });
}
payloadContent MissShader(const closesthit& str, parentedRays& nextgen, exindicesWithHead* terminates, sptr<tlas> ptlas) {
	using evec3 = Eigen::Vector3<halff>;
	using namespace half_float::literal;

	evec3 direction(str.r.way().data());
	evec3 light(0.0_h, -1.0_h, 0.0_h);

	halff doter = direction.dot(-light.normalized());
	doter = std::max(0.0_h, doter);
	doter = pow(doter, 2.0_h);

	terminates->push_head(str.r.index());

	halff amb = +0.0_h;//����
	return payloadContent({ 1.0_h,1.0_h ,1.0_h }, { doter + amb,doter + amb,doter + amb });
}