#include"general.hpp"
#include"sub.hpp"
#include "int24.hpp"

using namespace std;
using namespace Eigen;
//using namespace half_float::literal;


using evec3 = Eigen::Vector3<halff>;
using Affine3h = Eigen::Transform<halff, 3, 2>;

constexpr size_t MAX_GENERATIONS = 20;

constexpr size_t CORE_NUM = 1;
constexpr size_t CAMERA_RESOLUTION = 1024;
const halff CAMERA_FOV = halff(60. * std::numbers::pi / 180.0);
const extern sindex RAYNUM_LIMIT_BRUNCH = 1;//��{�̃��C���琶���镪��̍ő�l
constexpr exindex RAYNUM_LIMIT_GENERATION = (CAMERA_RESOLUTION * CAMERA_RESOLUTION);//�ꐢ��̃��C�̍ő吔

const extern exindex RAYNUM_LIMIT_ALL = RAYNUM_LIMIT_GENERATION * MAX_GENERATIONS*2;//�S����̃��C�̍��v�̍ő吔
constexpr exindex RAYNUM_LIMIT_TERMINATES = RAYNUM_LIMIT_GENERATION*2;//�I�[���C�̍ő吔

constexpr size_t BLASNUM_LIMIT = 8;//blas�̐��̐����@���ꂾ���̃I�u�W�F�N�g��z�u�ł���
const extern size_t ATTRIBUTE_SIZE = 3;//�A�g���r���[�g�̑傫��[word]
constexpr size_t TRIANGLES_NUM_LIMIT = 2048*32;//triangle�̍ő吔 ���ꂾ����triangle��p�ӂł���


const halff IGNORE_NEARHIT = 0.01;//���C�̓����蔻���t=����ȍ~�Ŕ�������@���ߌ���ʉ߂�����Ƃ��ɕK�v
const halff IGNORE_PARALLELHIT = 0.0;//norm dot direction������ȉ��̂Ƃ��i���[�t�F�[�Y�������@�J�����O�ƕ��s���̖�������������

const halff	TRIANGLE_EXTEND_SIZE = 32.0;//�|���S���̃T�C�Y���C�v�V�����̂���{�����g������ �傫���قǃi���[�t�F�[�Y���Â��Ȃ�
const halff AABB_TIMES_MARGINE = 0.0;//vsAABB�̌������Ԃ̃}�[�W���@�傫���قǃu���[�h�t�F�[�Y���Â��Ȃ�



#include "shaders.cpp"
const std::vector<std::tuple<string, hmat4,toolkit::materializer<RAYNUM_LIMIT_ALL, RAYNUM_LIMIT_BRUNCH,toolkit::attributeFramework<ATTRIBUTE_SIZE>>::shader>> model_gen = {
	std::make_tuple("../monkey.dae",DiagonalMatrix<halff,4>(.3,.3,.3,1.)*Affine3h(Translation<halff,3>(evec3(-0.0,0.0,0.0))).matrix(),GetColorShader<RAYNUM_LIMIT_ALL, RAYNUM_LIMIT_BRUNCH, ATTRIBUTE_SIZE>(hvec3(1., 0., 0.))),
	//std::make_tuple("../cube.dae",Affine3h(Translation<halff,3>(evec3(0.0_h,0.0_h,-2.0_h))).matrix(),HitMirror),
	//std::make_tuple("../wave.dae",Affine3h(Translation<halff,3>(evec3(0.0,-3.0,0.0))).matrix(),HitMirror)
};
/*
tlas���A�E�^�[����Ȃ�Ƃ��\�z���@����Ƀ��C�g���[�X�������s�����Ƃ�rayHierarchy�ɕϊ��@����������������邱�ƂŃt���[�����쐬����
�A�E�^�[����R�}���h���g���ăJ������blas�Ƃ��ꂼ��̕ϊ��𑗐M�@tlas���\������
*/

//���u����
struct _machines {
	toolkit::memoryCollection<payload, RAYNUM_LIMIT_ALL, RAYNUM_LIMIT_GENERATION, RAYNUM_LIMIT_TERMINATES, ATTRIBUTE_SIZE, BLASNUM_LIMIT, TRIANGLES_NUM_LIMIT> memory;

	toolkit::generator<RAYNUM_LIMIT_ALL, payload> generator;
	toolkit::broadphaser<CORE_NUM> broadphaser;
	toolkit::narrowphaser narrowphaser;
	toolkit::obstructer<RAYNUM_LIMIT_GENERATION> obstructer;
	toolkit::materializer<RAYNUM_LIMIT_ALL, RAYNUM_LIMIT_BRUNCH, toolkit::attributeFramework<ATTRIBUTE_SIZE>> materializer;
	toolkit::developer<payload, CAMERA_RESOLUTION> developer;

	_machines() :broadphaser(AABB_TIMES_MARGINE), narrowphaser(TRIANGLE_EXTEND_SIZE) {}
};

//�菇0(pre-phase)  �A�E�^�[�ōs����f�[�^�\���̏���
struct prephaseRez {
	std::vector<sptr<blas>> objs;
	sptr<camera> cam;
};
prephaseRez PrePhase() {
	//SetMaxWH(CAMERA_RESOLUTION, CAMERA_RESOLUTION);//bmp�ɉ𑜓x��ݒ�
	prephaseRez rez;
	//�菇0(���O����) ���f����ǂݍ����blas���쐬����&�J�������쐬����
	size_t trianglesnum_sum = 0;
	for (const auto& p : model_gen) {
		dmod tempmod;
		ModLoader(std::get<0>(p), tempmod);
		trianglesnum_sum += tempmod.size();
		sptr<blas> obj(new blas(tempmod));
		rez.objs.push_back(obj);
	}
	if (trianglesnum_sum > TRIANGLES_NUM_LIMIT)throw std::out_of_range("Triangle included LAS are more than TRIANGLES_NUM_LIMIT");

	//���̃A�g���r���[�g
	/*for (size_t i=0;i<rez.objs.size();i++)
		MakeDammyAttributes<toolkit::attributeFramework<ATTRIBUTE_SIZE>>(rez.objs.at(i).get()->triangles.size(),machines->memory.GetAttributes()->at(i));*/

	//�J�����𐶐�
	rez.cam.reset(new camera(CAMERA_RESOLUTION, CAMERA_RESOLUTION, -camera::CalcDistFromFov(CAMERA_FOV)));

	return rez;
}

//�菇1(upload-phase) blas��J�������O���{�ɓo�^��rooter��broad/narrow-phaser�ɓo�^(�O���{�����ł�tlas��0-gen ���C���쐻�����)
struct regphaseRez {
	sptr<las> scene;
};
void RegObjs(const vector<sptr<blas>>& objs, _machines* machines) {
	for (int i = 0; i < objs.size(); i++) {
		machines->memory.GetLAS()->push_back(make_pair(std::get<1>(model_gen.at(i)).inverse(), objs.at(i)));//blas�Ƃ��̕ϊ���o�^
		machines->materializer.mats.push_back(std::get<2>(model_gen.at(i)));
	}
}
void RegRays(const sptr<camera>& cam, _machines* machines) {
	machines->generator.RegisterRays(*cam, machines->memory.GetAllowsAllgen(), machines->memory.GetPayloadsAllgen());
}

int main() {
	auto preRez = PrePhase();//�A�E�^�[���f�[�^��p�ӂ���(blas��cam)

	//���Ƀw�b�_��ǂݍ���
	const std::string dicheaderPrefix = R"(C:\local\user\lensball\lensball\resultsX\HexBall\projRefRayMap)";
	projRefraDicHeader dicheader;
	{
		ifstream headerIfs(dicheaderPrefix+".head", std::ios::binary);
		cereal::BinaryInputArchive i_arc(headerIfs);
		i_arc(dicheader);
	}

	//���ʂ̃t���[���͂����ɕۑ�����
	const std::string frameSavePrefix = R"(C:\local\user\lensball\lensball\resultsX\projectorFrames\frame)";

	constexpr size_t rayTraceThreadNum = 4;
	std::array<uptr<std::thread>, rayTraceThreadNum> rayTraThreads;
	std::array<bool, rayTraceThreadNum> rayTraIsFin;
	const auto rayTracePipeline = [&](const size_t sceneid, decltype(rayTraIsFin)::iterator isfin) {
		//�}�V���𐶐�
		uptr<_machines> machines = make_unique<_machines>();
		RegObjs(preRez.objs, machines.get());//�A�E�^�[���f�[�^���O���{�ɓo�^(rooter��cam->0 gen rays���Abroadphaser��narrowphaser��blas-es->tlas��)

		//�V�[�����ƂɃ��C�g���[�V���O���s���K�v������@�܂背�C���X�g����ɓ���
		std::list<arrow3> raylist;
		{
			ifstream headerIfs(dicheaderPrefix + ".part" + to_string(sceneid), std::ios::binary);
			cereal::BinaryInputArchive i_arc(headerIfs);
			i_arc(raylist);
		}

		//������J�����ɂ���
		cout << dicheader.horizontalRes  << endl;
		auto raylistite = raylist.cbegin();
		sptr<camera> cam = make_shared<camera>(dicheader.horizontalRes * dicheader.verticalRes);//�s���ƂɃf�[�^�������Ă����̂ɋC������
		for (size_t hd = 0; hd < dicheader.horizontalRes; hd++)
			for (size_t vd = 0; vd < dicheader.verticalRes; vd++) {
				auto& target=cam->at(hd + vd * dicheader.horizontalRes);
				target.org() = hvec3((*raylistite).org().x(), (*raylistite).org().y(), (*raylistite).org().z());
				target.way() = hvec3(-(*raylistite).dir().x(), -(*raylistite).dir().y(), -(*raylistite).dir().z());

				raylistite++;
			}

		RegRays(cam, machines.get());
		machines->materializer.mats.miss = MissShader;

	

		for (size_t gen = 0; gen < MAX_GENERATIONS; gen++) {
			exindex genhead;//�S����id�Ɛ����id�̃I�t�Z�b�g
			exindex gensize;//������̃T�C�Y
			auto generation = machines->generator.GetGeneration(genhead, machines->memory.GetAllowsAllgen(), gensize);//rooter���琢����󂯎��

			machines->obstructer.InstallGeneration(generation, genhead, machines->memory.GetClosesthitsGen());//anyhit�ɍ�����̏���ʒm���Ă�����
			cout << "Broadphase began" << endl;
			auto bpRez = machines->broadphaser.Broadphase(generation, machines->memory.GetLAS());//�u���[�h�t�F�[�Y���s���@�U�z��������ray,g-index���ʂ𓾂�

			cout << "narrowphase began" << endl;
			auto npRez = machines->narrowphaser.RayTrace(*bpRez, IGNORE_NEARHIT, IGNORE_PARALLELHIT, machines->memory.GetLAS());//�u���[�h�t�F�[�Y���ʂ���i���[�t�F�[�Y���s��

			cout << "Obstructer phase began" << endl;
			exindex anyhitsize = machines->obstructer.Anyhit(*npRez, genhead, machines->memory.GetClosesthitsGen());//���C�̎Օ����v�Z��closest-hit���v�Z����@�����ł͐����id���g���Ă���̂Œ���

			cout << "Materializer began" << endl;
			parentedRays nextgen;
			machines->materializer.Shading(*machines->memory.GetClosesthitsGen(), nextgen, gen + 1 < MAX_GENERATIONS, machines->memory.GetPayloadsAllgen(), machines->memory.GetTerminatesGen(), gensize, machines->memory.GetLAS(), machines->memory.GetAttributes());//���C�̕\�ʂł̐U�镑�����v�Z next gen rays�𐶐�
			machines->generator.RegisterRays(nextgen, machines->memory.GetAllowsAllgen(), machines->memory.GetPayloadsAllgen());

			cout << "\t" << gen << "th generation report\n"
				<< "\t\tgensize= " << generation.size() << "\n"
				<< "\t\tbroadphase hits num: " << bpRez->size() << "\n"
				<< "\t\tnarrowphase hits num: " << npRez->size() << "\n"
				<< "\t\tclosests num: " << anyhitsize << "\n"
				<< "\t\tnext generation size: " << nextgen.size() << "\n\n"
				<< "\t\tterminates size: " << machines->memory.GetTerminatesGen()->head << "\n";

			//�����ヌ�C���������Ȃ��Ȃ�I��
			if (nextgen.empty())break;
		}

		//���ʂ��쐬
		cout << "developping began" << endl;
		auto pixels = machines->developer.Develop(machines->memory.GetPayloadsAllgen(), machines->memory.GetTerminatesGen());//���C�q�G�����L�[���猻������

		cout << "It is going to be completly soon..." << endl;
		PrintBmpWithAnotherSize_YOU_MUST_READ_COMMENT<CAMERA_RESOLUTION>(frameSavePrefix+to_string(sceneid)+".bmp", *pixels,dicheader.horizontalRes,dicheader.verticalRes);

		*isfin = true;
	};


	//�����X���b�h�ɉ�]�p�x��ς��Ȃ��犄�蓖�Ă�
	for (size_t rdgen = 0; rdgen < dicheader.rotationRes; rdgen++) {
		std::cout << "scene: " << rdgen << endl;
		//����rdgen�ł̏������J���Ă���X���b�h�Ɋ���t������
		bool isfound = false;
		while (!isfound) {//����t�����Ȃ���ΌJ��Ԃ�
			for (size_t th = 0; th < rayTraceThreadNum; th++)
				if (!rayTraThreads.at(th)) {//�󂫂Ȃ犄�t
					if (!isfound) {//��̃C���f�b�N�X�ɂ͈�񂾂�����t����
						isfound = true;
						rayTraIsFin.at(th) = false;//�t���O���N���A����

						rayTraThreads.at(th).reset(new std::thread(rayTracePipeline, rdgen, rayTraIsFin.begin() + th));//�X���b�h���s�J�n
					}
				}
				else if (rayTraIsFin.at(th)) {//�󂢂ĂȂ��ďI����Ă�Ȃ�
					rayTraThreads.at(th).get()->join();
					rayTraThreads.at(th).release();//���\�[�X���J��
				}
		}
	}

	//�S�X���b�h�̏I����҂�
	for (size_t thd = 0; thd < rayTraceThreadNum;thd++) {
		if (rayTraThreads.at(thd)) {
			rayTraThreads.at(thd)->join();
			rayTraThreads.at(thd).release();
		}
	}

	return 0;
}