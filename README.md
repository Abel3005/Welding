# Welding
## Description
WeldingProgram
====
## Version
### v1
Description:Service 통신을 통한 YOLO 3D XYZ 받아오기: COMPLETE
그럼에도 그라운드 데이터와 테스트 데이터간에 오차가 심함 
-> 그라운드 데이터가 잘못됬을 가능성이 있음
용접 로봇을 용접 포인트로 이동 시킴 데이터 전달
->(T_X,T_Y,T_Z)
그라운드 데이터를 얻을 때 사용한 TF 변환 공식에서의 TRANS를 출력시킴
->(G_X, G_Y, G_Z)
(T_X,T_Y,T_Z) = (0.803868, -0.055952, 0.596993)
(G_X,G_Y,G_Z) = (0.705381,-0.055894, 0.615535)
오류 해결 해야함
