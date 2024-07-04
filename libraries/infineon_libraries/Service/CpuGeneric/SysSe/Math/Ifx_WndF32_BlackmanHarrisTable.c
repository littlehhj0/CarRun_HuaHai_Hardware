/**
 * \file Ifx_WndF32_BlackmanHarrisTable.c
 * \brief Lookup table for Blackman-Harris window function
 *
 *
 * \version disabled
 * \copyright Copyright (c) 2013 Infineon Technologies AG. All rights reserved.
 *
 *
 *                                 IMPORTANT NOTICE
 *
 *
 * Use of this file is subject to the terms of use agreed between (i) you or 
 * the company in which ordinary course of business you are acting and (ii) 
 * Infineon Technologies AG or its licensees. If and as long as no such 
 * terms of use are agreed, use of this file is subject to following:


 * Boost Software License - Version 1.0 - August 17th, 2003

 * Permission is hereby granted, free of charge, to any person or 
 * organization obtaining a copy of the software and accompanying 
 * documentation covered by this license (the "Software") to use, reproduce,
 * display, distribute, execute, and transmit the Software, and to prepare
 * derivative works of the Software, and to permit third-parties to whom the 
 * Software is furnished to do so, all subject to the following:

 * The copyright notices in the Software and this entire statement, including
 * the above license grant, this restriction and the following disclaimer, must
 * be included in all copies of the Software, in whole or in part, and all
 * derivative works of the Software, unless such copies or derivative works are
 * solely in the form of machine-executable object code generated by a source
 * language processor.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE 
 * FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
���ڼ�����޷������޳����źŽ���FFT���㣬�������ǽ�ȡ���޳����н��з��������������������Ƶ������й¶��
���ô���������ȡ�ź��ܹ�����Ƶ������й¶��
��ͬ�Ĵ��������Ƶ��й¶������ͬ������Ч�������У����Ҵ��������õ��԰����Ժͼ򵥵ı�����ʽ���������Blackman���Ҵ�������
���ԣ�
������ҷ�ֵ��
�԰����С
˥���ٶȿ�
 *
 */

#include "Ifx_WndF32.h"

#if IFX_WNDF32_TABLE_LENGTH != 1024
#error This was generated with IFX_WNDF32_TABLE_LENGTH = 1024. Please regenerate.
#endif

CONST_CFG float32 Ifx_g_WndF32_blackmanHarrisTable[(IFX_WNDF32_TABLE_LENGTH / 2)] = {
    _DATAF(0.000060000000000001),
    _DATAF(6.05336435769415E-05),
    _DATAF(6.21351621010909E-05),
    _DATAF(6.48063189517042E-05),
    _DATAF(6.85500530901977E-05),
    _DATAF(7.33704790552552E-05),
    _DATAF(7.92728869546643E-05),
    _DATAF(8.62637424542366E-05),
    _DATAF(9.43506867646833E-05),
    _DATAF(0.000103542536623254),
    _DATAF(0.000113849284272844),
    _DATAF(0.00012528209743509),
    _DATAF(0.000137853319279022),
    _DATAF(0.000151576468382483),
    _DATAF(0.000166466238687386),
    _DATAF(0.000182538499446352),
    _DATAF(0.000199810295160321),
    _DATAF(0.000218299845506252),
    _DATAF(0.000238026545253333),
    _DATAF(0.000259010964166398),
    _DATAF(0.000281274846895671),
    _DATAF(0.000304841112851225),
    _DATAF(0.000329733856060103),
    _DATAF(0.000355978345005534),
    _DATAF(0.000383601022445701),
    _DATAF(0.00041262950521132),
    _DATAF(0.000443092583978778),
    _DATAF(0.000475020223018621),
    _DATAF(0.000508443559916298),
    _DATAF(0.000543394905263906),
    _DATAF(0.000579907742319714),
    _DATAF(0.000618016726635444),
    _DATAF(0.000657757685646449),
    _DATAF(0.000699167618224639),
    _DATAF(0.000742284694190907),
    _DATAF(0.000787148253784491),
    _DATAF(0.000833798807087666),
    _DATAF(0.000882278033402194),
    _DATAF(0.000932628780575385),
    _DATAF(0.000984895064273818),
    _DATAF(0.00103912206720055),
    _DATAF(0.00109535613825471),
    _DATAF(0.00115364479162906),
    _DATAF(0.00121403670584417),
    _DATAF(0.00127658172271511),
    _DATAF(0.00134133084624859),
    _DATAF(0.0014083362414667),
    _DATAF(0.00147765123315459),
    _DATAF(0.00154933030452976),
    _DATAF(0.0016234290958281),
    _DATAF(0.00170000440280525),
    _DATAF(0.00177911417514856),
    _DATAF(0.00186081751479774),
    _DATAF(0.00194517467416997),
    _DATAF(0.00203224705428634),
    _DATAF(0.00212209720279637),
    _DATAF(0.0022147888118974),
    _DATAF(0.00231038671614482),
    _DATAF(0.00240895689015029),
    _DATAF(0.00251056644616419),
    _DATAF(0.00261528363153874),
    _DATAF(0.00272317782606849),
    _DATAF(0.0028343195392042),
    _DATAF(0.00294878040713723),
    _DATAF(0.00306663318975018),
    _DATAF(0.00318795176743033),
    _DATAF(0.00331281113774309),
    _DATAF(0.00344128741195982),
    _DATAF(0.00357345781143924),
    _DATAF(0.00370940066385603),
    _DATAF(0.00384919539927517),
    _DATAF(0.00399292254606638),
    _DATAF(0.00414066372665728),
    _DATAF(0.00429250165311901),
    _DATAF(0.00444852012258346),
    _DATAF(0.00460880401248608),
    _DATAF(0.00477343927563203),
    _DATAF(0.0049425129350816),
    _DATAF(0.00511611307885149),
    _DATAF(0.00529432885442795),
    _DATAF(0.00547725046308894),
    _DATAF(0.00566496915403105),
    _DATAF(0.00585757721829767),
    _DATAF(0.00605516798250564),
    _DATAF(0.00625783580236627),
    _DATAF(0.00646567605599707),
    _DATAF(0.00667878513702144),
    _DATAF(0.00689726044745254),
    _DATAF(0.00712120039035792),
    _DATAF(0.00735070436230157),
    _DATAF(0.00758587274556064),
    _DATAF(0.00782680690011242),
    _DATAF(0.00807360915538975),
    _DATAF(0.00832638280180106),
    _DATAF(0.00858523208201078),
    _DATAF(0.0088502621819799),
    _DATAF(0.00912157922176039),
    _DATAF(0.00939929024604285),
    _DATAF(0.00968350321445337),
    _DATAF(0.00997432699159716),
    _DATAF(0.0102718713368455),
    _DATAF(0.010576246893865),
    _DATAF(0.0108875651798833),
    _DATAF(0.0112059385746926),
    _DATAF(0.0115314803093847),
    _DATAF(0.011864304454818),
    _DATAF(0.0122045259098121),
    _DATAF(0.0125522603890683),
    _DATAF(0.0129076244108144),
    _DATAF(0.0132707352841705),
    _DATAF(0.013641711096234),
    _DATAF(0.0140206706988822),
    _DATAF(0.0144077336952905),
    _DATAF(0.0148030204261634),
    _DATAF(0.0152066519556773),
    _DATAF(0.0156187500571341),
    _DATAF(0.0160394371983217),
    _DATAF(0.0164688365265826),
    _DATAF(0.0169070718535868),
    _DATAF(0.0173542676398092),
    _DATAF(0.0178105489787098),
    _DATAF(0.0182760415806149),
    _DATAF(0.0187508717562994),
    _DATAF(0.0192351664002677),
    _DATAF(0.0197290529737345),
    _DATAF(0.020232659487302),
    _DATAF(0.0207461144833353),
    _DATAF(0.0212695470180336),
    _DATAF(0.0218030866431987),
    _DATAF(0.0223468633876972),
    _DATAF(0.0229010077386208),
    _DATAF(0.0234656506221394),
    _DATAF(0.0240409233840514),
    _DATAF(0.0246269577700285),
    _DATAF(0.0252238859055558),
    _DATAF(0.025831840275569),
    _DATAF(0.026450953703786),
    _DATAF(0.0270813593317373),
    _DATAF(0.0277231905974923),
    _DATAF(0.0283765812140843),
    _DATAF(0.0290416651476349),
    _DATAF(0.029718576595178),
    _DATAF(0.0304074499621848),
    _DATAF(0.031108419839792),
    _DATAF(0.0318216209817327),
    _DATAF(0.0325471882809733),
    _DATAF(0.0332852567460565),
    _DATAF(0.0340359614771537),
    _DATAF(0.0347994376418264),
    _DATAF(0.0355758204505013),
    _DATAF(0.036365245131659),
    _DATAF(0.0371678469067388),
    _DATAF(0.0379837609647635),
    _DATAF(0.038813122436685),
    _DATAF(0.039656066369453),
    _DATAF(0.0405127276998116),
    _DATAF(0.041383241227825),
    _DATAF(0.0422677415901346),
    _DATAF(0.0431663632329524),
    _DATAF(0.0440792403847932),
    _DATAF(0.0450065070289477),
    _DATAF(0.0459482968757025),
    _DATAF(0.0469047433343071),
    _DATAF(0.0478759794846953),
    _DATAF(0.0488621380489625),
    _DATAF(0.0498633513626023),
    _DATAF(0.0508797513455092),
    _DATAF(0.0519114694727488),
    _DATAF(0.0529586367451015),
    _DATAF(0.0540213836593831),
    _DATAF(0.0550998401785474),
    _DATAF(0.0561941357015757),
    _DATAF(0.0573043990331573),
    _DATAF(0.0584307583531662),
    _DATAF(0.0595733411859395),
    _DATAF(0.0607322743693613),
    _DATAF(0.06190768402376),
    _DATAF(0.0630996955206202),
    _DATAF(0.0643084334511193),
    _DATAF(0.0655340215944914),
    _DATAF(0.0667765828862246),
    _DATAF(0.0680362393860989),
    _DATAF(0.0693131122460692),
    _DATAF(0.0706073216779996),
    _DATAF(0.0719189869212565),
    _DATAF(0.0732482262101651),
    _DATAF(0.0745951567413363),
    _DATAF(0.0759598946408714),
    _DATAF(0.0773425549314486),
    _DATAF(0.0787432514993022),
    _DATAF(0.0801620970610963),
    _DATAF(0.0815992031307037),
    _DATAF(0.083054679985896),
    _DATAF(0.0845286366349502),
    _DATAF(0.0860211807831818),
    _DATAF(0.0875324187994092),
    _DATAF(0.0890624556823589),
    _DATAF(0.0906113950270166),
    _DATAF(0.092179338990934),
    _DATAF(0.0937663882604969),
    _DATAF(0.0953726420171649),
    _DATAF(0.0969981979036867),
    _DATAF(0.0986431519903029),
    _DATAF(0.100307598740941),
    _DATAF(0.101991630979414),
    _DATAF(0.103695339855624),
    _DATAF(0.105418814811791),
    _DATAF(0.107162143548701),
    _DATAF(0.108925411991991),
    _DATAF(0.110708704258479),
    _DATAF(0.112512102622539),
    _DATAF(0.114335687482543),
    _DATAF(0.116179537327367),
    _DATAF(0.118043728702971),
    _DATAF(0.119928336179073),
    _DATAF(0.121833432315909),
    _DATAF(0.123759087631096),
    _DATAF(0.125705370566615),
    _DATAF(0.127672347455906),
    _DATAF(0.129660082491093),
    _DATAF(0.131668637690355),
    _DATAF(0.133698072865431),
    _DATAF(0.135748445589297),
    _DATAF(0.137819811163995),
    _DATAF(0.139912222588642),
    _DATAF(0.142025730527626),
    _DATAF(0.144160383278985),
    _DATAF(0.146316226742996),
    _DATAF(0.14849330439097),
    _DATAF(0.150691657234271),
    _DATAF(0.152911323793562),
    _DATAF(0.155152340068285),
    _DATAF(0.157414739506395),
    _DATAF(0.15969855297435),
    _DATAF(0.162003808727361),
    _DATAF(0.164330532379924),
    _DATAF(0.166678746876629),
    _DATAF(0.169048472463272),
    _DATAF(0.171439726658257),
    _DATAF(0.173852524224323),
    _DATAF(0.176286877140581),
    _DATAF(0.178742794574886),
    _DATAF(0.181220282856548),
    _DATAF(0.183719345449389),
    _DATAF(0.186239982925158),
    _DATAF(0.188782192937316),
    _DATAF(0.191345970195187),
    _DATAF(0.193931306438502),
    _DATAF(0.196538190412331),
    _DATAF(0.199166607842421),
    _DATAF(0.201816541410937),
    _DATAF(0.204487970732631),
    _DATAF(0.207180872331433),
    _DATAF(0.209895219617483),
    _DATAF(0.212630982864606),
    _DATAF(0.21538812918824),
    _DATAF(0.218166622523828),
    _DATAF(0.220966423605681),
    _DATAF(0.223787489946321),
    _DATAF(0.226629775816306),
    _DATAF(0.229493232224558),
    _DATAF(0.232377806899193),
    _DATAF(0.235283444268855),
    _DATAF(0.238210085444584),
    _DATAF(0.241157668202195),
    _DATAF(0.244126126965209),
    _DATAF(0.247115392788316),
    _DATAF(0.25012539334139),
    _DATAF(0.253156052894067),
    _DATAF(0.256207292300887),
    _DATAF(0.259279028987005),
    _DATAF(0.262371176934486),
    _DATAF(0.265483646669187),
    _DATAF(0.268616345248233),
    _DATAF(0.271769176248094),
    _DATAF(0.27494203975327),
    _DATAF(0.278134832345589),
    _DATAF(0.281347447094127),
    _DATAF(0.284579773545761),
    _DATAF(0.287831697716344),
    _DATAF(0.291103102082533),
    _DATAF(0.294393865574252),
    _DATAF(0.297703863567812),
    _DATAF(0.301032967879694),
    _DATAF(0.30438104676098),
    _DATAF(0.307747964892468),
    _DATAF(0.311133583380456),
    _DATAF(0.314537759753201),
    _DATAF(0.317960347958064),
    _DATAF(0.321401198359353),
    _DATAF(0.324860157736845),
    _DATAF(0.328337069285016),
    _DATAF(0.331831772612977),
    _DATAF(0.335344103745104),
    _DATAF(0.338873895122394),
    _DATAF(0.342420975604529),
    _DATAF(0.345985170472656),
    _DATAF(0.3495663014329),
    _DATAF(0.353164186620592),
    _DATAF(0.356778640605235),
    _DATAF(0.360409474396196),
    _DATAF(0.364056495449141),
    _DATAF(0.367719507673201),
    _DATAF(0.371398311438886),
    _DATAF(0.375092703586736),
    _DATAF(0.378802477436722),
    _DATAF(0.382527422798394),
    _DATAF(0.386267325981779),
    _DATAF(0.39002196980903),
    _DATAF(0.393791133626826),
    _DATAF(0.397574593319535),
    _DATAF(0.401372121323116),
    _DATAF(0.4051834866398),
    _DATAF(0.409008454853506),
    _DATAF(0.412846788146031),
    _DATAF(0.416698245313995),
    _DATAF(0.420562581786534),
    _DATAF(0.424439549643771),
    _DATAF(0.428328897636028),
    _DATAF(0.432230371203805),
    _DATAF(0.436143712498517),
    _DATAF(0.440068660403987),
    _DATAF(0.444004950558688),
    _DATAF(0.447952315378757),
    _DATAF(0.451910484081743),
    _DATAF(0.455879182711121),
    _DATAF(0.459858134161553),
    _DATAF(0.463847058204895),
    _DATAF(0.46784567151695),
    _DATAF(0.471853687704974),
    _DATAF(0.475870817335914),
    _DATAF(0.479896767965388),
    _DATAF(0.483931244167404),
    _DATAF(0.487973947564809),
    _DATAF(0.492024576860471),
    _DATAF(0.496082827869183),
    _DATAF(0.500148393550297),
    _DATAF(0.50422096404107),
    _DATAF(0.508300226690728),
    _DATAF(0.512385866095238),
    _DATAF(0.516477564132795),
    _DATAF(0.520575),
    _DATAF(0.524677850248738),
    _DATAF(0.528785788823753),
    _DATAF(0.532898487100904),
    _DATAF(0.537015613926103),
    _DATAF(0.541136835654929),
    _DATAF(0.545261816192918),
    _DATAF(0.549390217036503),
    _DATAF(0.553521697314626),
    _DATAF(0.557655913830994),
    _DATAF(0.561792521106979),
    _DATAF(0.565931171425163),
    _DATAF(0.570071514873506),
    _DATAF(0.574213199390142),
    _DATAF(0.578355870808795),
    _DATAF(0.582499172904798),
    _DATAF(0.586642747441718),
    _DATAF(0.590786234218577),
    _DATAF(0.594929271117651),
    _DATAF(0.599071494152859),
    _DATAF(0.603212537518709),
    _DATAF(0.607352033639815),
    _DATAF(0.611489613220962),
    _DATAF(0.615624905297715),
    _DATAF(0.619757537287565),
    _DATAF(0.623887135041594),
    _DATAF(0.628013322896669),
    _DATAF(0.632135723728129),
    _DATAF(0.636253959002978),
    _DATAF(0.640367648833561),
    _DATAF(0.644476412031722),
    _DATAF(0.648579866163425),
    _DATAF(0.652677627603835),
    _DATAF(0.656769311592843),
    _DATAF(0.660854532291033),
    _DATAF(0.664932902836069),
    _DATAF(0.669004035399499),
    _DATAF(0.673067541243964),
    _DATAF(0.6771230307808),
    _DATAF(0.681170113628019),
    _DATAF(0.685208398668663),
    _DATAF(0.689237494109519),
    _DATAF(0.693257007540177),
    _DATAF(0.697266545992425),
    _DATAF(0.701265715999973),
    _DATAF(0.705254123658479),
    _DATAF(0.70923137468589),
    _DATAF(0.713197074483052),
    _DATAF(0.717150828194615),
    _DATAF(0.721092240770188),
    _DATAF(0.72502091702575),
    _DATAF(0.7289364617053),
    _DATAF(0.732838479542731),
    _DATAF(0.736726575323917),
    _DATAF(0.740600353949007),
    _DATAF(0.744459420494891),
    _DATAF(0.748303380277859),
    _DATAF(0.752131838916412),
    _DATAF(0.755944402394215),
    _DATAF(0.759740677123202),
    _DATAF(0.763520270006787),
    _DATAF(0.76728278850319),
    _DATAF(0.771027840688865),
    _DATAF(0.774755035321995),
    _DATAF(0.778463981906066),
    _DATAF(0.782154290753498),
    _DATAF(0.785825573049304),
    _DATAF(0.789477440914794),
    _DATAF(0.793109507471277),
    _DATAF(0.796721386903776),
    _DATAF(0.800312694524721),
    _DATAF(0.803883046837616),
    _DATAF(0.807432061600672),
    _DATAF(0.810959357890378),
    _DATAF(0.814464556165),
    _DATAF(0.817947278328014),
    _DATAF(0.821407147791419),
    _DATAF(0.824843789538964),
    _DATAF(0.82825683018923),
    _DATAF(0.831645898058594),
    _DATAF(0.835010623224019),
    _DATAF(0.8383506375857),
    _DATAF(0.841665574929516),
    _DATAF(0.844955070989297),
    _DATAF(0.848218763508878),
    _DATAF(0.851456292303937),
    _DATAF(0.854667299323595),
    _DATAF(0.85785142871177),
    _DATAF(0.861008326868267),
    _DATAF(0.864137642509598),
    _DATAF(0.867239026729501),
    _DATAF(0.87031213305917),
    _DATAF(0.87335661752716),
    _DATAF(0.876372138718963),
    _DATAF(0.879358357836245),
    _DATAF(0.882314938755719),
    _DATAF(0.885241548087659),
    _DATAF(0.888137855234021),
    _DATAF(0.891003532446169),
    _DATAF(0.8938382548822),
    _DATAF(0.896641700663836),
    _DATAF(0.899413550932886),
    _DATAF(0.90215348990726),
    _DATAF(0.90486120493652),
    _DATAF(0.907536386556961),
    _DATAF(0.9101787285462),
    _DATAF(0.912787927977272),
    _DATAF(0.915363685272211),
    _DATAF(0.917905704255112),
    _DATAF(0.92041369220465),
    _DATAF(0.922887359906061),
    _DATAF(0.925326421702555),
    _DATAF(0.927730595546164),
    _DATAF(0.930099603048002),
    _DATAF(0.932433169527932),
    _DATAF(0.93473102406363),
    _DATAF(0.936992899539027),
    _DATAF(0.939218532692123),
    _DATAF(0.941407664162166),
    _DATAF(0.943560038536173),
    _DATAF(0.945675404394792),
    _DATAF(0.947753514357501),
    _DATAF(0.949794125127105),
    _DATAF(0.951796997533564),
    _DATAF(0.953761896577093),
    _DATAF(0.955688591470572),
    _DATAF(0.957576855681214),
    _DATAF(0.959426466971513),
    _DATAF(0.961237207439446),
    _DATAF(0.963008863557916),
    _DATAF(0.964741226213449),
    _DATAF(0.966434090744106),
    _DATAF(0.968087256976625),
    _DATAF(0.969700529262773),
    _DATAF(0.971273716514903),
    _DATAF(0.972806632240705),
    _DATAF(0.974299094577144),
    _DATAF(0.975750926323582),
    _DATAF(0.977161954974065),
    _DATAF(0.978532012748779),
    _DATAF(0.979860936624663),
    _DATAF(0.981148568365171),
    _DATAF(0.982394754549168),
    _DATAF(0.98359934659898),
    _DATAF(0.984762200807557),
    _DATAF(0.985883178364764),
    _DATAF(0.986962145382792),
    _DATAF(0.987998972920675),
    _DATAF(0.988993537007913),
    _DATAF(0.989945718667196),
    _DATAF(0.990855403936222),
    _DATAF(0.991722483888599),
    _DATAF(0.992546854653837),
    _DATAF(0.993328417436418),
    _DATAF(0.994067078533934),
    _DATAF(0.994762749354304),
    _DATAF(0.995415346432055),
    _DATAF(0.996024791443661),
    _DATAF(0.996591011221945),
    _DATAF(0.997113937769534),
    _DATAF(0.997593508271372),
    _DATAF(0.998029665106271),
    _DATAF(0.99842235585752),
    _DATAF(0.998771533322532),
    _DATAF(0.999077155521532),
    _DATAF(0.99933918570529),
    _DATAF(0.999557592361884),
    _DATAF(0.999732349222509),
    _DATAF(0.999863435266309),
    _DATAF(0.999950834724255),
    _DATAF(0.999994537082046),
};