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
constexpr size_t CAMERA_RESOLUTION = 768;
const halff CAMERA_FOV = halff(60. * std::numbers::pi / 180.0);
const extern sindex RAYNUM_LIMIT_BRUNCH = 1;//��{�̃��C���琶���镪��̍ő�l
constexpr exindex RAYNUM_LIMIT_GENERATION = (CAMERA_RESOLUTION * CAMERA_RESOLUTION);//�ꐢ��̃��C�̍ő吔

const extern exindex RAYNUM_LIMIT_ALL = RAYNUM_LIMIT_GENERATION * MAX_GENERATIONS*2;//�S����̃��C�̍��v�̍ő吔
constexpr exindex RAYNUM_LIMIT_TERMINATES = RAYNUM_LIMIT_GENERATION*2;//�I�[���C�̍ő吔

constexpr size_t BLASNUM_LIMIT = 8;//blas�̐��̐����@���ꂾ���̃I�u�W�F�N�g��z�u�ł���
const extern size_t ATTRIBUTE_SIZE = 3;//�A�g���r���[�g�̑傫��[word]
constexpr size_t TRIANGLES_NUM_LIMIT = 2048*32;//triangle�̍ő吔 ���ꂾ����triangle��p�ӂł���


const halff IGNORE_NEARHIT = 0.01;//���C�̓����蔻���t=����ȍ~�Ŕ�������@���ߌ���ʉ߂�����Ƃ��ɕK�v
const halff IGNORE_PARALLELHIT = cos(89.99/180.*std::numbers::pi);//norm dot direction������ȉ��̂Ƃ��i���[�t�F�[�Y�������@�J�����O�ƕ��s���̖�������������

const halff	TRIANGLE_EXTEND_SIZE = 32.0;//�|���S���̃T�C�Y���C�v�V�����̂���{�����g������ �傫���قǃi���[�t�F�[�Y���Â��Ȃ�
const halff AABB_TIMES_MARGINE = 0.0;//vsAABB�̌������Ԃ̃}�[�W���@�傫���قǃu���[�h�t�F�[�Y���Â��Ȃ�



#include "shaders.cpp"
const std::vector<std::tuple<string, hmat4,toolkit::materializer<RAYNUM_LIMIT_ALL, RAYNUM_LIMIT_BRUNCH,toolkit::attributeFramework<ATTRIBUTE_SIZE>>::shader>> model_gen = {
	//std::make_tuple("../monkey.dae",DiagonalMatrix<halff,4>(.3,.3,.3,1.)*Affine3h(Translation<halff,3>(evec3(-0.0,0.0,0.0))).matrix(),GetColorShader<RAYNUM_LIMIT_ALL, RAYNUM_LIMIT_BRUNCH, ATTRIBUTE_SIZE>(hvec3(1., 0., 0.))),
	std::make_tuple("../minicube.dae",Eigen::Matrix4d::Identity(),HitNormColor)//,
	//std::make_tuple("../wave.dae",Affine3h(Translation<halff,3>(evec3(0.0,-3.0,0.0))).matrix(),HitMirror)
};
/*
tlas���A�E�^�[����Ȃ�Ƃ��\�z���@����Ƀ��C�g���[�X�������s�����Ƃ�rayHierarchy�ɕϊ��@����������������邱�ƂŃt���[�����쐬����
�A�E�^�[����R�}���h���g���ăJ������blas�Ƃ��ꂼ��̕ϊ��𑗐M�@tlas���\������
*/


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

int main() {

	//���Ƀw�b�_��ǂݍ���
	const std::string dicheaderPrefix = R"(C:\local\user\lensball\lensball\rez\SimVis\dic\dic)";
	projRefraDicHeader dicheader;
	{
		ifstream headerIfs(dicheaderPrefix+".head", std::ios::binary);
		cereal::BinaryInputArchive i_arc(headerIfs);
		i_arc(dicheader);
	}

	//���ʂ̃t���[���͂����ɕۑ�����
	const std::string frameSavePrefix = R"(C:\local\user\lensball\lensball\rez\SimVis\frames\frame)";

	constexpr size_t rayTraceThreadNum = 18;
	std::array<uptr<std::thread>, rayTraceThreadNum> rayTraThreads;
	std::array<bool, rayTraceThreadNum> rayTraIsFin;
	const auto rayTracePipeline = [&](const size_t sceneid, decltype(rayTraIsFin)::iterator isfin) {
		//�V�[�����ƂɃ��C�g���[�V���O���s���K�v������@�܂背�C���X�g����ɓ���
		std::list<arrow3> raylist;
		{
			ifstream headerIfs(dicheaderPrefix + ".part" + to_string(sceneid), std::ios::binary);
			cereal::BinaryInputArchive i_arc(headerIfs);
			i_arc(raylist);
		}

		//������J�����ɂ���
		auto raylistite = raylist.cbegin();
		sptr<camera> cam = make_shared<camera>();//�s���ƂɃf�[�^�������Ă����̂ɋC������
		std::vector<std::pair<size_t, size_t>> pixPoses;
		exindex rayIdCount = 0;
		for (size_t vd = 0; vd < dicheader.verticalRes; vd++) {
			for (size_t hd = 0; hd < dicheader.horizontalRes; hd++, raylistite++) {
				//�[���x�N�g�������邱�Ƃ�����
				if ((*raylistite).dir() == uvec<3>::Zero()) {
					//�����ꍇ�]������K�v���Ȃ����Ă���
					continue;
				}

				cam->push_back(ray());
				pixPoses.push_back(make_pair(hd, vd));

				auto& target = cam->back();
				target.org() = hvec3((*raylistite).org().x(), (*raylistite).org().y(), (*raylistite).org().z());
				target.way() = hvec3(-(*raylistite).dir().x(), -(*raylistite).dir().y(), -(*raylistite).dir().z());
				target.indexed(rayIdCount++);//index�͍��W�����Ɍ��߂ꂻ��
			}
		}
		//���f����p��
		size_t trianglesnum_sum = 0;
		std::list<sptr<blas>> lases;
		uptr<las> world = make_unique<las>();
		for (const auto& p : model_gen) {
			dmod tempmod;
			ModLoader(std::get<0>(p), tempmod);
			trianglesnum_sum += tempmod.size();
			sptr<blas> obj(new blas(tempmod));
			world->push_back(make_pair(std::get<1>(p), obj));
		}
		if (trianglesnum_sum > TRIANGLES_NUM_LIMIT)throw std::out_of_range("Triangle included LAS are more than TRIANGLES_NUM_LIMIT");

		//�u���[�h�t�F�[�Y
		toolkit::broadphaser<1> bper(AABB_TIMES_MARGINE);
		const auto bpResult = bper.Broadphase(*cam, world.get());
		//�i���[�t�F�[�Y
		toolkit::narrowphaser nper(TRIANGLE_EXTEND_SIZE);
		const auto npResult = nper.RayTrace(*bpResult, IGNORE_NEARHIT, IGNORE_PARALLELHIT, world.get());

		toolkit::obstructer<RAYNUM_LIMIT_GENERATION> obser;
		std::unordered_map<exindex, closesthit> closests;
		obser.Anyhit(*npResult, 0, closests);

		//�����t�@�C�������[�h���ĕ������}�b�s���O����
		sptr<bitmapx> colors = make_shared<bitmapx>(dicheader.horizontalRes * dicheader.verticalRes);
		for (auto& c : *colors)
			c = hvec3({ 1.,1.,1. });//�\������Ȃ�������
		exindex count = 0;
		for (const auto& i : *cam) {
			const auto thispos = pixPoses.at(count);
			auto ite = closests.find(count);
			colors->at(thispos.first + thispos.second * dicheader.horizontalRes) = { 0.,0.,ite != closests.end() ? 1. : 0. };

			count++;
		}

		PrintBmpWithAnotherSize_YOU_MUST_READ_COMMENT(frameSavePrefix+to_string(sceneid)+".bmp", *colors,dicheader.horizontalRes,dicheader.verticalRes);

		//���ʂ��쐬
		cout << "developped" << endl;
		//dever.Develop()
		//auto pixels = machines->developer.Develop(machines->memory.GetPayloadsAllgen(), machines->memory.GetTerminatesGen());//���C�q�G�����L�[���猻������

		//cout << "It is going to be completly soon..." << endl;
		//PrintBmpWithAnotherSize_YOU_MUST_READ_COMMENT<CAMERA_RESOLUTION>(frameSavePrefix+to_string(sceneid)+".bmp", *pixels,dicheader.horizontalRes,dicheader.verticalRes);

		*isfin = true;
	};
	//rayTracePipeline(0, rayTraIsFin.begin());

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