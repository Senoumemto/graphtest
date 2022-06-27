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
constexpr size_t CAMERA_RESOLUTION = 256;
const halff CAMERA_FOV = halff(120.0 * std::numbers::pi / 180.0);
const extern sindex RAYNUM_LIMIT_BRUNCH = 1;//一本のレイから生じる分岐の最大値
constexpr exindex RAYNUM_LIMIT_GENERATION = (CAMERA_RESOLUTION * CAMERA_RESOLUTION);//一世代のレイの最大数

const extern exindex RAYNUM_LIMIT_ALL = RAYNUM_LIMIT_GENERATION * MAX_GENERATIONS*2;//全世代のレイの合計の最大数
constexpr exindex RAYNUM_LIMIT_TERMINATES = RAYNUM_LIMIT_GENERATION*2;//終端レイの最大数

constexpr size_t BLASNUM_LIMIT = 8;//blasの数の制限　これだけのオブジェクトを配置できる
const extern size_t ATTRIBUTE_SIZE = 3;//アトリビュートの大きさ[word]
constexpr size_t TRIANGLES_NUM_LIMIT = 2048;//triangleの最大数 これだけのtriangleを用意できる


const halff IGNORE_NEARHIT = 0.01_h;//レイの当たり判定はt=これ以降で発生する　透過光を通過させるときに必要
const halff IGNORE_PARALLELHIT = 0.0_h;//norm dot directionがこれ以下のときナローフェーズが発生　カリングと並行光の無視が発生する

const halff	TRIANGLE_EXTEND_SIZE = 32.0_h;//ポリゴンのサイズをイプシロンのこれ倍だけ拡張する 大きいほどナローフェーズが甘くなる
const halff AABB_TIMES_MARGINE = 0.0_h;//vsAABBの交差時間のマージン　大きいほどブロードフェーズが甘くなる



#include "shaders.cpp"
const std::vector<std::tuple<string, hmat4,toolkit::materializer<RAYNUM_LIMIT_ALL, RAYNUM_LIMIT_BRUNCH,toolkit::attributeFramework<ATTRIBUTE_SIZE>>::shader>> model_gen = {
	std::make_tuple("../monkey.dae",Affine3h(Translation<halff,3>(evec3(-0.0_h,0.0_h,-2.1_h))).matrix(),HitMirror),
	//std::make_tuple("../cube.dae",Affine3h(Translation<halff,3>(evec3(0.0_h,0.0_h,-2.0_h))).matrix(),HitMirror),
	//std::make_tuple("../wave.dae",Affine3h(Translation<halff,3>(evec3(0.0_h,-3.0_h,0.0_h))).matrix(),HitMirror)
};
/*
tlasをアウターからなんとか構築し　それにレイトレース処理を行うことでrayHierarchyに変換　それを現像処理することでフレームを作成する
アウターからコマンドを使ってカメラとblasとそれぞれの変換を送信　tlasを構成する
*/

//装置たち
struct _machines{
	toolkit::memoryCollection<payload, RAYNUM_LIMIT_ALL,RAYNUM_LIMIT_GENERATION, RAYNUM_LIMIT_TERMINATES,ATTRIBUTE_SIZE,BLASNUM_LIMIT,TRIANGLES_NUM_LIMIT> memory;

	toolkit::generator<RAYNUM_LIMIT_ALL,payload> generator;
	toolkit::broadphaser<CORE_NUM> broadphaser;
	toolkit::narrowphaser narrowphaser;
	toolkit::obstructer<RAYNUM_LIMIT_GENERATION> obstructer;
	toolkit::materializer<RAYNUM_LIMIT_ALL,RAYNUM_LIMIT_BRUNCH,toolkit::attributeFramework<ATTRIBUTE_SIZE>> materializer;
	toolkit::developer<payload, CAMERA_RESOLUTION> developer;

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
	size_t trianglesnum_sum = 0;
	for (const auto& p : model_gen) {
		dmod tempmod;
		ModLoader(std::get<0>(p), tempmod);
		trianglesnum_sum += tempmod.size();
		sptr<blas> obj(new blas(tempmod));
		rez.objs.push_back(obj);
	}
	if (trianglesnum_sum > TRIANGLES_NUM_LIMIT)throw std::out_of_range("Triangle included LAS are more than TRIANGLES_NUM_LIMIT");

	//仮のアトリビュート
	for (size_t i=0;i<rez.objs.size();i++)
		MakeDammyAttributes<toolkit::attributeFramework<ATTRIBUTE_SIZE>>(rez.objs.at(i).get()->triangles.size(),machines.memory.GetAttributes()->at(i));

	//カメラを生成
	rez.cam.reset(new camera(CAMERA_RESOLUTION, CAMERA_RESOLUTION, -camera::CalcDistFromFov(CAMERA_FOV)));

	return rez;
}

//手順1(upload-phase) blasやカメラをグラボに登録しrooterやbroad/narrow-phaserに登録(グラボ内部ではtlasと0-gen レイが作製される)
struct regphaseRez {
	sptr<las> scene;
};
void RegPhase(const vector<sptr<blas>>& objs, const sptr<camera>& cam) {

	//tlasを作製
	for (int i = 0; i < objs.size(); i++) {
		machines.memory.GetLAS()->push_back(make_pair(std::get<1>(model_gen.at(i)).inverse(), objs.at(i)));//blasとその変換を登録
		machines.materializer.mats.push_back(std::get<2>(model_gen.at(i)));
	}


	machines.generator.RegisterRays(*cam, machines.memory.GetAllowsAllgen(), machines.memory.GetPayloadsAllgen());

	////シェーダーをインストール
	//machines.materialer.mats.push_back(HitMirror);
	//machines.materialer.mats.push_back(HitMirror);

	machines.materializer.mats.miss = MissShader;
}

int main() {

	auto preRez = PrePhase();//アウターがデータを用意する(blasとcam)
	RegPhase(preRez.objs, preRez.cam);//アウターがデータをグラボに登録(rooterへcam->0 gen raysが、broadphaserとnarrowphaserへblas-es->tlasが)

	for (size_t gen = 0; gen < MAX_GENERATIONS; gen++) {
		exindex genhead;//全世代idと世代内idのオフセット
		exindex gensize;//今世代のサイズ
		auto generation = machines.generator.GetGeneration(genhead, machines.memory.GetAllowsAllgen(), gensize);//rooterから世代を受け取る

		machines.obstructer.InstallGeneration(generation, genhead, machines.memory.GetClosesthitsGen());//anyhitに今世代の情報を通知してあげる
		cout << "Broadphase began" << endl;
		auto bpRez = machines.broadphaser.Broadphase(generation,machines.memory.GetLAS());//ブロードフェーズを行う　偽陽性を持つray,g-index結果を得る

		cout << "narrowphase began" << endl;
		auto npRez = machines.narrowphaser.RayTrace(*bpRez, IGNORE_NEARHIT, IGNORE_PARALLELHIT,machines.memory.GetLAS());//ブロードフェーズ結果からナローフェーズを行う

		cout << "Obstructer phase began" << endl;
		exindex anyhitsize=machines.obstructer.Anyhit(*npRez, genhead, machines.memory.GetClosesthitsGen());//レイの遮蔽を計算しclosest-hitを計算する　ここでは世代内idを使っているので注意

		cout << "Materializer began" << endl;
		parentedRays nextgen;
		machines.materializer.Shading(*machines.memory.GetClosesthitsGen(), nextgen, gen + 1 < MAX_GENERATIONS, machines.memory.GetPayloadsAllgen(), machines.memory.GetTerminatesGen(), gensize, machines.memory.GetLAS(),machines.memory.GetAttributes());//レイの表面での振る舞いを計算 next gen raysを生成
		machines.generator.RegisterRays(nextgen, machines.memory.GetAllowsAllgen(),machines.memory.GetPayloadsAllgen());

		cout << "\t" << gen << "th generation report\n"
			<< "\t\tgensize= " << generation.size() << "\n"
			<< "\t\tbroadphase hits num: " << bpRez->size() << "\n"
			<< "\t\tnarrowphase hits num: " << npRez->size() << "\n"
			<< "\t\tclosests num: " << anyhitsize << "\n"
			<< "\t\tnext generation size: " << nextgen.size() << "\n\n"
			<< "\t\tterminates size: " << machines.memory.GetTerminatesGen()->head << "\n";

		//次世代レイが発生しないなら終了
		if (nextgen.empty())break;
	}


	cout << "developping began" << endl;
	auto pixels = machines.developer.Develop(machines.memory.GetPayloadsAllgen(), machines.memory.GetTerminatesGen());//レイヒエラルキーから現像する

	cout << "It is going to be completly soon..." << endl;
	PrintBmp<CAMERA_RESOLUTION>("out.bmp", *pixels);

	return 0;
}