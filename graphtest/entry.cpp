
#include"general.hpp"
#include"sub.hpp"
#include "int24.hpp"

using namespace std;
using namespace Eigen;

#define CORE_NUM 1
#define ROOTER_CACHE (128*128)
#define ANYHIT_CACHE (ROOTER_CACHE)
#define MATERIALER_CACHE (ROOTER_CACHE)
#define CAMERA_RESOLUTION 128

/*
tlas���A�E�^�[����Ȃ�Ƃ��\�z���@����Ƀ��C�g���[�X�������s�����Ƃ�rayHierarchy�ɕϊ��@����������������邱�ƂŃt���[�����쐬����
�A�E�^�[����R�}���h���g���ăJ������blas�Ƃ��ꂼ��̕ϊ��𑗐M�@tlas���\������
*/

//���u����
struct {
	toolkit::rooter<ROOTER_CACHE,payload> rooter;
	toolkit::broadphaser<CORE_NUM> broadphaser;
	toolkit::narrowphaser narrowphaser;
	toolkit::anyhit<ANYHIT_CACHE> anyhit;
	toolkit::materialer<MATERIALER_CACHE> materialer;
	toolkit::developper<payload, CAMERA_RESOLUTION> developper;
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
	ModLoader("../monkey.dae", model);
	//MakeTestSquare(model);
	sptr<blas> obj(new blas(model));
	rez.objs.push_back(obj);


	rez.cam.reset(new camera(CAMERA_RESOLUTION, CAMERA_RESOLUTION, -1.0));

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


int main() {

	auto preRez = PrePhase();//�A�E�^�[���f�[�^��p�ӂ���(blas��cam)


	RegPhase(preRez.objs, preRez.cam);//�A�E�^�[���f�[�^���O���{�ɓo�^(rooter��cam->0 gen rays���Abroadphaser��narrowphaser��blas-es->tlas��)

	index genhead;
	auto generation = machines.rooter.GetGeneration(genhead);//rooter���琢����󂯎��

	machines.anyhit.InstallGeneration(generation, genhead);//anyhit�ɍ�����̏���ʒm���Ă�����
	cout << "Broadphase began" << endl;
	auto bpRez = machines.broadphaser.Broadphase(generation);//�u���[�h�t�F�[�Y���s���@�U�z��������ray,g-index���ʂ𓾂�

	cout << "narrowphase began" << endl;
	auto npRez = machines.narrowphaser.RayTrace(*bpRez);//�u���[�h�t�F�[�Y���ʂ���i���[�t�F�[�Y���s��

	cout << "anyhit phase began" << endl;
	auto closestHits = machines.anyhit.Anyhit(*npRez, genhead);//���C�̎Օ����v�Z��closest-hit���v�Z����@�����ł͐����id���g���Ă���̂Œ���

	cout << "shading began" << endl;
	auto rayPayloads=machines.materialer.Shading(*closestHits);//���C�̕\�ʂł̐U�镑�����v�Z next gen rays�𐶐�
	machines.rooter.AddPayloads(*rayPayloads,genhead);

	cout << "developping began" << endl;
	auto pixels = machines.developper.Develop(*machines.rooter.GetHierarchy());//���C�q�G�����L�[���猻������

	cout << "It is going to be completly soon..." << endl;
	PrintBmp<CAMERA_RESOLUTION>("out.bmp", *pixels);

	return 0;
}