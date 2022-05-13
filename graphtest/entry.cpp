
#include"general.hpp"
#include"sub.hpp"
#include "int24.hpp"

using namespace std;
using namespace Eigen;
using namespace half_float::literal;
using Affine3h = Eigen::Transform<halff, 3, 2>;

constexpr size_t MAX_GENERATIONS = 10;

constexpr size_t CORE_NUM = 1;
constexpr size_t CAMERA_RESOLUTION = 512;
constexpr exindex RAYNUM_LIMIT_ALL = (CAMERA_RESOLUTION * CAMERA_RESOLUTION * 3);
constexpr exindex RAYNUM_LIMIT_GENERATION = (CAMERA_RESOLUTION * CAMERA_RESOLUTION);
constexpr exindex RAYNUM_LIMIT_TERMINATES = RAYNUM_LIMIT_GENERATION;


const halff IGNORE_NEARHIT = 0.01_h;

const string MODEL_PATH = "../ico.dae";

/*
tlas���A�E�^�[����Ȃ�Ƃ��\�z���@����Ƀ��C�g���[�X�������s�����Ƃ�rayHierarchy�ɕϊ��@����������������邱�ƂŃt���[�����쐬����
�A�E�^�[����R�}���h���g���ăJ������blas�Ƃ��ꂼ��̕ϊ��𑗐M�@tlas���\������
*/

//���u����
struct {
	toolkit::memoryCollection<payload, RAYNUM_LIMIT_ALL,RAYNUM_LIMIT_GENERATION, RAYNUM_LIMIT_TERMINATES> memory;

	toolkit::rooter<RAYNUM_LIMIT_ALL,payload> rooter;
	toolkit::broadphaser<CORE_NUM> broadphaser;
	toolkit::narrowphaser narrowphaser;
	toolkit::anyhit<RAYNUM_LIMIT_GENERATION> anyhit;
	toolkit::materialer<RAYNUM_LIMIT_ALL> materialer;
	toolkit::developper<payload, CAMERA_RESOLUTION> developper;
}machines;

payload HitShader(const closesthit& att, rays& nextgen, exindicesWithHead* terminates, sptr<tlas> ptlas);
payload MissShader(const closesthit& str, rays& nextgen, exindicesWithHead* terminates, sptr<tlas> ptlas);
#include "shaders.cpp"


//�菇0(pre-phase)  �A�E�^�[�ōs����f�[�^�\���̏���
struct prephaseRez {
	std::vector<sptr<blas>> objs;
	sptr<camera> cam;
};
prephaseRez PrePhase() {

	SetMaxWH(CAMERA_RESOLUTION, CAMERA_RESOLUTION);//bmp�ɉ𑜓x��ݒ�

	prephaseRez rez;
	//�菇0(���O����) ���f����ǂݍ����blas���쐬����&�J�������쐬����
	dmod model;
	ModLoader(MODEL_PATH, model);
	sptr<blas> obj(new blas(model));
	rez.objs.push_back(obj);

	//�J�����𐶐�
	rez.cam.reset(new camera(CAMERA_RESOLUTION, CAMERA_RESOLUTION, -1.0));

	return rez;
}


//�菇1(upload-phase) blas��J�������O���{�ɓo�^��rooter��broad/narrow-phaser�ɓo�^(�O���{�����ł�tlas��0-gen ���C���쐻�����)
struct regphaseRez {
	sptr<tlas> scene;
};
void RegPhase(const vector<sptr<blas>>& objs, const sptr<camera>& cam) {
	using evec3 = Eigen::Vector3<halff>;

	//tlas���쐻
	sptr<tlas> scene(new tlas);
	for (const auto& obj : objs)
		scene->push_back(make_pair(Affine3h(Eigen::Translation<halff, 3>(evec3(0.0_h, 0.0_h, 0.0_h))).matrix().inverse(), obj));//blas�Ƃ��̕ϊ���o�^


	machines.rooter.RegisterRays(*cam, machines.memory.GetAllGenRays());

	//�g���[�T�[��tlas������ɃC���X�g�[��
	machines.broadphaser.ptlas = scene;
	machines.narrowphaser.ptlas = scene;
	machines.materialer.ptlas = scene;

	//�V�F�[�_�[���C���X�g�[��
	machines.materialer.HitShader = HitShader;
	machines.materialer.MissShader = MissShader;
}


int main() {

	auto preRez = PrePhase();//�A�E�^�[���f�[�^��p�ӂ���(blas��cam)


	RegPhase(preRez.objs, preRez.cam);//�A�E�^�[���f�[�^���O���{�ɓo�^(rooter��cam->0 gen rays���Abroadphaser��narrowphaser��blas-es->tlas��)

	for (size_t gen = 0; gen < MAX_GENERATIONS; gen++) {
		exindex genhead;//�S����id�Ɛ����id�̃I�t�Z�b�g
		exindex gensize;//������̃T�C�Y
		auto generation = machines.rooter.GetGeneration(genhead, machines.memory.GetAllGenRays(), gensize);//rooter���琢����󂯎��

		machines.anyhit.InstallGeneration(generation, genhead, machines.memory.GetNowGenClosests());//anyhit�ɍ�����̏���ʒm���Ă�����
		cout << "Broadphase began" << endl;
		auto bpRez = machines.broadphaser.Broadphase(generation);//�u���[�h�t�F�[�Y���s���@�U�z��������ray,g-index���ʂ𓾂�

		cout << "narrowphase began" << endl;
		auto npRez = machines.narrowphaser.RayTrace(*bpRez, IGNORE_NEARHIT);//�u���[�h�t�F�[�Y���ʂ���i���[�t�F�[�Y���s��

		cout << "anyhit phase began" << endl;
		exindex anyhitsize=machines.anyhit.Anyhit(*npRez, genhead, machines.memory.GetNowGenClosests());//���C�̎Օ����v�Z��closest-hit���v�Z����@�����ł͐����id���g���Ă���̂Œ���

		cout << "shading began" << endl;
		rays nextgen;
		machines.materialer.Shading(*machines.memory.GetNowGenClosests(), nextgen, machines.memory.GetAllGenPayloads(), machines.memory.GetTerminates(), gensize);//���C�̕\�ʂł̐U�镑�����v�Z next gen rays�𐶐�
		machines.rooter.RegisterRays(nextgen, machines.memory.GetAllGenRays());

		cout << "\t" << gen << "th generation report\n"
			<< "\t\tgensize= " << generation.size() << "\n"
			<< "\t\tbroadphase hits num: " << bpRez->size() << "\n"
			<< "\t\tnarrowphase hits num: " << npRez->size() << "\n"
			<< "\t\tanyhits num: " << anyhitsize << "\n"
			<< "\t\tnext generation size: " << nextgen.size() << "\n\n"
			<< "\t\tterminates size: " << machines.memory.GetTerminates()->head << "\n";

		//�����ヌ�C���������Ȃ��Ȃ�I��
		if (nextgen.empty())break;
	}


	cout << "developping began" << endl;
	auto pixels = machines.developper.Develop(machines.memory.GetAllGenPayloads(), machines.memory.GetTerminates());//���C�q�G�����L�[���猻������

	cout << "It is going to be completly soon..." << endl;
	PrintBmp<CAMERA_RESOLUTION>("out.bmp", *pixels);

	return 0;
}