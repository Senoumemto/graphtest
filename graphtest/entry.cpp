
#include"general.hpp"
#include"sub.hpp"

using namespace std;
using namespace Eigen;

#define CORE_NUM 1
#define ROOTER_CACHE (32*32)
#define ANYHIT_CACHE (ROOTER_CACHE)
#define MATERIALER_CACHE (ROOTER_CACHE)
#define CAMERA_RESOLUTION_SQRT 32

/*
tlas���A�E�^�[����Ȃ�Ƃ��\�z���@����Ƀ��C�g���[�X�������s�����Ƃ�rayHierarchy�ɕϊ��@����������������邱�ƂŃt���[�����쐬����
�A�E�^�[����R�}���h���g���ăJ������blas�Ƃ��ꂼ��̕ϊ��𑗐M�@tlas���\������
*/

//���u����
struct {
	toolkit::rooter<ROOTER_CACHE> rooter;
	toolkit::broadphaser<CORE_NUM> broadphaser;
	toolkit::narrowphaser narrowphaser;
	toolkit::anyhit<ANYHIT_CACHE> anyhit;
	toolkit::materialer<MATERIALER_CACHE> materialer;
}machines;


//�菇0(pre-phase)  �A�E�^�[�ōs����f�[�^�\���̏���
struct prephaseRez {
	std::vector<sptr<blas>> objs;
	sptr<camera> cam;
};
prephaseRez PrePhase() {
	prephaseRez rez;
	//�菇0(���O����) ���f����ǂݍ����blas���쐬����&�J�������쐬����
	dmod model;
	//ModLoader("../monkey.dae", model);
	MakeTestSquare(model);
	sptr<blas> obj(new blas(model));
	rez.objs.push_back(obj);


	rez.cam.reset(new camera(CAMERA_RESOLUTION_SQRT, CAMERA_RESOLUTION_SQRT, -1.0));

	return rez;
}


//�菇1(upload-phase) blas��J�������O���{�ɓo�^��rooter��broad/narrow-phaser�ɓo�^(�O���{�����ł�tlas��0-gen ���C���쐻�����)
struct regphaseRez {
	sptr<tlas> scene;
};
void RegPhase(const vector<sptr<blas>>& objs, const sptr<camera>& cam) {

	//tlas���쐻
	sptr<tlas> scene(new tlas);
	for (const auto& obj : objs)
		scene->push_back(make_pair(hmat4::Identity(), obj));//blas�Ƃ��̕ϊ���o�^

	machines.rooter.RegisterRays(*cam);

	//�g���[�T�[��tlas������ɃC���X�g�[��
	machines.broadphaser.ptlas = scene;

	machines.narrowphaser.ptlas = scene;
}

////�菇2(broadphase) �o�^���ꂽ�f�[�^��p���ău���[�h�t�F�[�Y���s��
//sptr<broadphaseResults> BroadPhase() {
//	//�菇3(broadphase)  broadphase���s���@���ʂƂ��ă��C�ƍL��Փˌ���(gindex)�̃y�A�̃��X�g���󂯎��
//	sptr<broadphaseResults> bprez(new broadphaseResults);//�L��Փˌ��ʃp�b�P�[�W
//	while (machines.rooter) {
//		ray r = machines.rooter.Get();//���C����z���Ă��炤
//
//		auto r_sHits = machines.broadphaser.Raytrace({ r });//�u���[�h�t�F�[�Y���s���L��Փˏ����󂯎��
//		for (const auto& h : r_sHits)
//			bprez->push_back(broadphaseResultElement(r, h));//�q�b�g�������ʃp�b�P�[�W�ɓ����
//	}
//
//	return bprez;
//}

int main() {

	auto preRez = PrePhase();//�A�E�^�[���f�[�^��p�ӂ���(blas��cam)


	RegPhase(preRez.objs, preRez.cam);//�A�E�^�[���f�[�^���O���{�ɓo�^(rooter��cam->0 gen rays���Abroadphaser��narrowphaser��blas-es->tlas��)


	//auto bpRez = BroadPhase();	//done���߂�����΃O���{���u���[�h�t�F�[�Y���s��
	auto generation = machines.rooter.GetGeneration();//rooter���琢����󂯎��

	//auto npRez = machines.narrowphaser.RayTrace(*bpRez);//�u���[�h�t�F�[�Y���ʂ���i���[�t�F�[�Y���s��
	//auto closestHits = machines.anyhit.Anyhit(*npRez);//���C�̎Օ����v�Z��closest-hit���v�Z����
	//auto rayPayloads=machines.materialer.Shading(*closestHits);//���C�̕\�ʂł̐U�镑�����v�Z next gen rays�𐶐�

	return 0;
}