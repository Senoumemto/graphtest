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
const extern sindex RAYNUM_LIMIT_BRUNCH = 1;//一本のレイから生じる分岐の最大値
constexpr exindex RAYNUM_LIMIT_GENERATION = (CAMERA_RESOLUTION * CAMERA_RESOLUTION);//一世代のレイの最大数

const extern exindex RAYNUM_LIMIT_ALL = RAYNUM_LIMIT_GENERATION * MAX_GENERATIONS*2;//全世代のレイの合計の最大数
constexpr exindex RAYNUM_LIMIT_TERMINATES = RAYNUM_LIMIT_GENERATION*2;//終端レイの最大数


const halff IGNORE_NEARHIT = 0.01_h;//レイの当たり判定はt=これ以降で発生する　透過光を通過させるときに必要
const halff IGNORE_PARALLELHIT = 0.0_h;//norm dot directionがこれ以下のときナローフェーズが発生　カリングと並行光の無視が発生する

const halff	TRIANGLE_EXTEND_SIZE = 32.0_h;//ポリゴンのサイズをイプシロンのこれ倍だけ拡張する 大きいほどナローフェーズが甘くなる
const halff AABB_TIMES_MARGINE = 0.0_h;//vsAABBの交差時間のマージン　大きいほどブロードフェーズが甘くなる



#include "shaders.cpp"
const std::vector<std::tuple<string, hmat4,toolkit::materialer<RAYNUM_LIMIT_ALL, RAYNUM_LIMIT_BRUNCH>::shader>> model_gen = {
	std::make_tuple("../monkey.dae",Affine3h(Translation<halff,3>(evec3(-0.0_h,0.0_h,1.1_h))).matrix(),HitLight),
	std::make_tuple("../cube.dae",Affine3h(Translation<halff,3>(evec3(0.0_h,0.0_h,-2.0_h))).matrix(),HitMirror),
	//std::make_tuple("../wave.dae",Affine3h(Translation<halff,3>(evec3(0.0_h,-3.0_h,0.0_h))).matrix(),HitMirror)
};
/*
tlasをアウターからなんとか構築し　それにレイトレース処理を行うことでrayHierarchyに変換　それを現像処理することでフレームを作成する
アウターからコマンドを使ってカメラとblasとそれぞれの変換を送信　tlasを構成する
*/

//装置たち
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


//手順0(pre-phase)  アウターで行われるデータ構造の準備
struct prephaseRez {
	std::vector<sptr<blas>> objs;
	sptr<camera> cam;
};
prephaseRez PrePhase() {

	SetMaxWH(CAMERA_RESOLUTION, CAMERA_RESOLUTION);//bmpに解像度を設定

	prephaseRez rez;
	//手順0(事前処理) モデルを読み込んでblasを作成する&カメラを作成する
	for (const auto& p : model_gen) {
		dmod tempmod;
		ModLoader(std::get<0>(p), tempmod);
		sptr<blas> obj(new blas(tempmod));
		rez.objs.push_back(obj);
	}

	//カメラを生成
	rez.cam.reset(new camera(CAMERA_RESOLUTION, CAMERA_RESOLUTION, -camera::CalcDistFromFov(CAMERA_FOV)));

	return rez;
}

//手順1(upload-phase) blasやカメラをグラボに登録しrooterやbroad/narrow-phaserに登録(グラボ内部ではtlasと0-gen レイが作製される)
struct regphaseRez {
	sptr<tlas> scene;
};
void RegPhase(const vector<sptr<blas>>& objs, const sptr<camera>& cam) {

	//tlasを作製
	sptr<tlas> scene(new tlas);
	for (int i = 0; i < objs.size(); i++) {
		scene->push_back(make_pair(std::get<1>(model_gen.at(i)).inverse(), objs.at(i)));//blasとその変換を登録
		machines.materialer.mats.push_back(std::get<2>(model_gen.at(i)));
	}


	machines.rooter.RegisterRays(*cam, machines.memory.GetAllGenRays(), machines.memory.GetAllGenPayloads());

	//トレーサーにtlasをそれにインストール
	machines.broadphaser.ptlas = scene;
	machines.narrowphaser.ptlas = scene;
	machines.materialer.ptlas = scene;

	////シェーダーをインストール
	//machines.materialer.mats.push_back(HitMirror);
	//machines.materialer.mats.push_back(HitMirror);

	machines.materialer.mats.miss = MissShader;
}

int main() {

	auto preRez = PrePhase();//アウターがデータを用意する(blasとcam)
	RegPhase(preRez.objs, preRez.cam);//アウターがデータをグラボに登録(rooterへcam->0 gen raysが、broadphaserとnarrowphaserへblas-es->tlasが)

	for (size_t gen = 0; gen < MAX_GENERATIONS; gen++) {
		exindex genhead;//全世代idと世代内idのオフセット
		exindex gensize;//今世代のサイズ
		auto generation = machines.rooter.GetGeneration(genhead, machines.memory.GetAllGenRays(), gensize);//rooterから世代を受け取る

		machines.anyhit.InstallGeneration(generation, genhead, machines.memory.GetNowGenClosests());//anyhitに今世代の情報を通知してあげる
		cout << "Broadphase began" << endl;
		auto bpRez = machines.broadphaser.Broadphase(generation);//ブロードフェーズを行う　偽陽性を持つray,g-index結果を得る

		cout << "narrowphase began" << endl;
		auto npRez = machines.narrowphaser.RayTrace(*bpRez, IGNORE_NEARHIT, IGNORE_PARALLELHIT);//ブロードフェーズ結果からナローフェーズを行う

		cout << "anyhit phase began" << endl;
		exindex anyhitsize=machines.anyhit.Anyhit(*npRez, genhead, machines.memory.GetNowGenClosests());//レイの遮蔽を計算しclosest-hitを計算する　ここでは世代内idを使っているので注意

		cout << "shading began" << endl;
		parentedRays nextgen;
		machines.materialer.Shading(*machines.memory.GetNowGenClosests(), nextgen, gen+1 < MAX_GENERATIONS, machines.memory.GetAllGenPayloads(), machines.memory.GetTerminates(), gensize);//レイの表面での振る舞いを計算 next gen raysを生成
		machines.rooter.RegisterRays(nextgen, machines.memory.GetAllGenRays(),machines.memory.GetAllGenPayloads());

		cout << "\t" << gen << "th generation report\n"
			<< "\t\tgensize= " << generation.size() << "\n"
			<< "\t\tbroadphase hits num: " << bpRez->size() << "\n"
			<< "\t\tnarrowphase hits num: " << npRez->size() << "\n"
			<< "\t\tanyhits num: " << anyhitsize << "\n"
			<< "\t\tnext generation size: " << nextgen.size() << "\n\n"
			<< "\t\tterminates size: " << machines.memory.GetTerminates()->head << "\n";

		//次世代レイが発生しないなら終了
		if (nextgen.empty())break;
	}


	cout << "developping began" << endl;
	auto pixels = machines.developper.Develop(machines.memory.GetAllGenPayloads(), machines.memory.GetTerminates());//レイヒエラルキーから現像する

	cout << "It is going to be completly soon..." << endl;
	PrintBmp<CAMERA_RESOLUTION>("out.bmp", *pixels);

	return 0;
}