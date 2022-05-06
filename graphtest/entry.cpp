
#include"general.hpp"
#include"sub.hpp"
#include "int24.hpp"

using namespace std;
using namespace Eigen;
using namespace half_float::literal;

constexpr size_t MAX_GENERATIONS = 10;

constexpr size_t CORE_NUM = 1;
constexpr size_t CAMERA_RESOLUTION = 1024;
constexpr exindex RAYNUM_LIMIT_ALL = (CAMERA_RESOLUTION * CAMERA_RESOLUTION * 3);
constexpr exindex RAYNUM_LIMIT_GENERATION = (CAMERA_RESOLUTION * CAMERA_RESOLUTION);
constexpr exindex RAYNUM_LIMIT_TERMINATES = RAYNUM_LIMIT_GENERATION;


const halff IGNORE_NEARHIT = 0.01_h;

const string MODEL_PATH = "../monkey.dae";

/*
tlasをアウターからなんとか構築し　それにレイトレース処理を行うことでrayHierarchyに変換　それを現像処理することでフレームを作成する
アウターからコマンドを使ってカメラとblasとそれぞれの変換を送信　tlasを構成する
*/

//装置たち
struct {
	toolkit::memoryCollection<payload, RAYNUM_LIMIT_ALL,RAYNUM_LIMIT_GENERATION, RAYNUM_LIMIT_TERMINATES> memory;

	toolkit::rooter<RAYNUM_LIMIT_ALL,payload> rooter;
	toolkit::broadphaser<CORE_NUM> broadphaser;
	toolkit::narrowphaser narrowphaser;
	toolkit::anyhit<RAYNUM_LIMIT_GENERATION> anyhit;
	toolkit::materialer<RAYNUM_LIMIT_ALL> materialer;
	toolkit::developper<payload, CAMERA_RESOLUTION> developper;
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
	dmod model;
	ModLoader(MODEL_PATH, model);
	//MakeTestSquare(model);
	sptr<blas> obj(new blas(model));
	rez.objs.push_back(obj);


	rez.cam.reset(new camera(CAMERA_RESOLUTION, CAMERA_RESOLUTION, -1.0));

	return rez;
}


//手順1(upload-phase) blasやカメラをグラボに登録しrooterやbroad/narrow-phaserに登録(グラボ内部ではtlasと0-gen レイが作製される)
struct regphaseRez {
	sptr<tlas> scene;
};
void RegPhase(const vector<sptr<blas>>& objs, const sptr<camera>& cam) {

	//メモリを接続
	//machines.rooter.ConnectMemory(machines.memory.GetAllGenRays());
	//machines.anyhit.ConnectMemmory(machines.memory.GetNowGenClosests());
	//machines.materialer.ConnectMemory(machines.memory.GetAllGenPayloads());
	//machines.developper.ConnectMemory(machines.memory.GetAllGenPayloads());

	//tlasを作製
	sptr<tlas> scene(new tlas);
	for (const auto& obj : objs)
		scene->push_back(make_pair(hmat4::Identity(), obj));//blasとその変換を登録

	machines.rooter.RegisterRays(*cam, machines.memory.GetAllGenRays());

	//トレーサーにtlasをそれにインストール
	machines.broadphaser.ptlas = scene;
	machines.narrowphaser.ptlas = scene;
	machines.materialer.ptlas = scene;

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
		auto npRez = machines.narrowphaser.RayTrace(*bpRez, IGNORE_NEARHIT);//ブロードフェーズ結果からナローフェーズを行う

		cout << "anyhit phase began" << endl;
		exindex anyhitsize=machines.anyhit.Anyhit(*npRez, genhead, machines.memory.GetNowGenClosests());//レイの遮蔽を計算しclosest-hitを計算する　ここでは世代内idを使っているので注意

		cout << "shading began" << endl;
		rays nextgen;
		machines.materialer.Shading(*machines.memory.GetNowGenClosests(), nextgen, machines.memory.GetAllGenPayloads(), machines.memory.GetTerminates(), gensize);//レイの表面での振る舞いを計算 next gen raysを生成
		machines.rooter.RegisterRays(nextgen, machines.memory.GetAllGenRays());

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
	auto pixels = machines.developper.Develop(machines.memory.GetAllGenPayloads());//レイヒエラルキーから現像する

	cout << "It is going to be completly soon..." << endl;
	PrintBmp<CAMERA_RESOLUTION>("out.bmp", *pixels);

	return 0;
}