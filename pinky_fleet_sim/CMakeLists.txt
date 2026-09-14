cmake_minimum_required(VERSION 3.8)
project(pinky_fleet_sim)

find_package(ament_cmake REQUIRED)

# 실행 코드는 없다. launch / 월드 / 맵 / 파라미터만 설치한다.
# params 와 map 안의 파일 일부는 scripts/ 의 생성기가 만든다.
install(DIRECTORY
  launch
  worlds
  map
  params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
