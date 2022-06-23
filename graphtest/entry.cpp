#include"general.hpp"
#include"sub.hpp"
#include "int24.hpp"

using namespace std;
using namespace Eigen;
using namespace half_float::literal;


using evec3 = Eigen::Vector3<halff>;
using Affine3h = Eigen::Transform<halff, 3, 2>;

constexpr size_t MAX_GENERATIONS = 20;

constexpr size_t CORE_NUM = 1;
constexpr size_t CAMERA_RESOLUTION = 512;
const halff CAMERA_FOV = halff(120.0 * std::numbers::pi / 180.0);
const extern sindex RAYNUM_LIMIT_BRUNCH = 1;//��{�̃��C���琶���镪��̍ő�l
constexpr exindex RAYNUM_LIMIT_GENERATION = (CAMERA_RESOLUTION * CAMERA_RESOLUTION);//�ꐢ��̃��C�̍ő吔

const extern exindex RAYNUM_LIMIT_ALL = RAYNUM_LIMIT_GENERATION * MAX_GENERATIONS*2;//�S����̃��C�̍��v�̍ő吔
constexpr exindex RAYNUM_LIMIT_TERMINATES = RAYNUM_LIMIT_GENERATION*2;//�I�[���C�̍ő吔


const halff IGNORE_NEARHIT = 0.01_h;//���C�̓����蔻���t=����ȍ~�Ŕ�������@���ߌ���ʉ߂�����Ƃ��ɕK�v
const halff IGNORE_PARALLELHIT = 0.0_h;//norm dot direction������ȉ��̂Ƃ��i���[�t�F�[�Y�������@�J�����O�ƕ��s���̖�������������

const halff	TRIANGLE_EXTEND_SIZE = 32.0_h;//�|���S���̃T�C�Y���C�v�V�����̂���{�����g������ �傫���قǃi���[�t�F�[�Y���Â��Ȃ�
const halff AABB_TIMES_MARGINE = 0.0_h;//vsAABB�̌������Ԃ̃}�[�W���@�傫���قǃu���[�h�t�F�[�Y���Â��Ȃ�



#include "shaders.cpp"
const std::vector<std::tuple<string, hmat4,toolkit::materialer<RAYNUM_LIMIT_ALL, RAYNUM_LIMIT_BRUNCH>::shader>> model_gen = {
	std::make_tuple("../monkey.dae",Affine3h(Translation<halff,3>(evec3(-0.0_h,0.0_h,1.1_h))).matrix(),HitLight),
	std::make_tuple("../cube.dae",Affine3h(Translation<halff,3>(evec3(0.0_h,0.0_h,-2.0_h))).matrix(),HitMirror),
	//std::make_tuple("../wave.dae",Affine3h(Translation<halff,3>(evec3(0.0_h,-3.0_h,0.0_h))).matrix(),HitMirror)
};
/*
tlas���A�E�^�[����Ȃ�Ƃ��\�z���@����Ƀ��C�g���[�X�������s�����Ƃ�rayHierarchy�ɕϊ��@����������������邱�ƂŃt���[�����쐬����
�A�E�^�[����R�}���h���g���ăJ������blas�Ƃ��ꂼ��̕ϊ��𑗐M�@tlas���\������
*/

//���u����
struct _machines{
	toolkit::memoryCollection<payload, RAYNUM_LIMIT_ALL,RAYNUM_LIMIT_GENERATION, RAYNUM_LIMIT_TERMINATES> memory;

	toolkit::rooter<RAYNUM_LIMIT_ALL,payload> rooter;
	toolkit::broadphaser<CORE_NUM> broadphaser;
	toolkit::narrowphaser narrowphaser;
	toolkit::anyhit<RAYNUM_LIMIT_GENERATION> anyhit;
	toolkit::materialer<RAYNUM_LIMIT_ALL,RAYNUM_LIMIT_BRUNCH> materialer;
	toolkit::developper<payload, CAMERA_RESOLUTION> developper;

	_machines():broadphaser(AABB_TIMES_MARGINE),narrowphaser(TRIANGLE_EXTEND_SIZE) {}
}machines;


//�菇0(pre-phase)  �A�E�^�[�ōs����f�[�^�\���̏���
struct prephaseRez {
	std::vector<sptr<blas>> objs;
	sptr<camera> cam;
};
prephaseRez PrePhase() {

	SetMaxWH(CAMERA_RESOLUTION, CAMERA_RESOLUTION);//bmp�ɉ𑜓x��ݒ�

	prephaseRez rez;
	//�菇0(���O����) ���f����ǂݍ����blas���쐬����&�J�������쐬����
	for (const auto& p : model_gen) {
		dmod tempmod;
		ModLoader(std::get<0>(p), tempmod);
		sptr<blas> obj(new blas(tempmod));
		rez.objs.push_back(obj);
	}

	//�J�����𐶐�
	rez.cam.reset(new camera(CAMERA_RESOLUTION, CAMERA_RESOLUTION, -camera::CalcDistFromFov(CAMERA_FOV)));

	return rez;
}

//�菇1(upload-phase) blas��J�������O���{�ɓo�^��rooter��broad/narrow-phaser�ɓo�^(�O���{�����ł�tlas��0-gen ���C���쐻�����)
struct regphaseRez {
	sptr<tlas> scene;
};
void RegPhase(const vector<sptr<blas>>& objs, const sptr<camera>& cam) {

	//tlas���쐻
	sptr<tlas> scene(new tlas);
	for (int i = 0; i < objs.size(); i++) {
		scene->push_back(make_pair(std::get<1>(model_gen.at(i)).inverse(), objs.at(i)));//blas�Ƃ��̕ϊ���o�^
		machines.materialer.mats.push_back(std::get<2>(model_gen.at(i)));
	}


	machines.rooter.RegisterRays(*cam, machines.memory.GetAllGenRays(), machines.memory.GetAllGenPayloads());

	//�g���[�T�[��tlas������ɃC���X�g�[��
	machines.broadphaser.ptlas = scene;
	machines.narrowphaser.ptlas = scene;
	machines.materialer.ptlas = scene;

	////�V�F�[�_�[���C���X�g�[��
	//machines.materialer.mats.push_back(HitMirror);
	//machines.materialer.mats.push_back(HitMirror);

	machines.materialer.mats.miss = MissShader;
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
		auto npRez = machines.narrowphaser.RayTrace(*bpRez, IGNORE_NEARHIT, IGNORE_PARALLELHIT);//�u���[�h�t�F�[�Y���ʂ���i���[�t�F�[�Y���s��

		cout << "anyhit phase began" << endl;
		exindex anyhitsize=machines.anyhit.Anyhit(*npRez, genhead, machines.memory.GetNowGenClosests());//���C�̎Օ����v�Z��closest-hit���v�Z����@�����ł͐����id���g���Ă���̂Œ���

		cout << "shading began" << endl;
		parentedRays nextgen;
		machines.materialer.Shading(*machines.memory.GetNowGenClosests(), nextgen, gen+1 < MAX_GENERATIONS, machines.memory.GetAllGenPayloads(), machines.memory.GetTerminates(), gensize);//���C�̕\�ʂł̐U�镑�����v�Z next gen rays�𐶐�
		machines.rooter.RegisterRays(nextgen, machines.memory.GetAllGenRays(),machines.memory.GetAllGenPayloads());

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