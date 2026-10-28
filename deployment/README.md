# 보드·외부 PC 실행 파일

`decision-ap` 폴더 전체를 복사할 때 다음 파일도 함께 옮긴다.

| 경로 | 내용 | 실행 장비 |
| --- | --- | --- |
| `images/board-arm64.tar.zst` | ARM64 Perception, System 포함 Control, Decision AP 통합 이미지 | BSP 보드 |
| `images/pc-rviz-amd64.tar.zst` | AMD64 Perception 이미지의 RViz2와 Autoware 플러그인 | 외부 PC |
| `map/sample-map-decision/` | Lanelet2·PointCloud 샘플 지도 | BSP 보드 |

압축 이미지는 Git과 Docker 빌드 컨텍스트에서 제외하지만 이 폴더 안에 실제 파일로 보관한다. 두 장비 모두 Docker와 `zstd`가 필요하다. 복사 후 저장소 루트에서 `sha256sum -c deployment/SHA256SUMS`로 확인한다. `run-katech-stack.sh`와 `run-remote-rviz.sh`는 필요한 이미지가 Docker에 없으면 각 압축 파일을 자동으로 불러온다. 이미 다른 버전의 같은 태그가 로컬에 있으면 자동으로 덮어쓰지 않으므로, 갱신 시에는 아래 명령으로 직접 불러온다.

```sh
# BSP 보드
zstd -dc deployment/images/board-arm64.tar.zst | docker load

# 외부 x86_64 PC
zstd -dc deployment/images/pc-rviz-amd64.tar.zst | docker load
```

이미지를 다시 빌드한 뒤 이 저장소 안의 사본을 갱신하려면 PC에서 실행한다. `board-arm64.tar.zst`에는 `decision-ap:local-aarch64`, `katech-autoware-system-control-arm64:adsw-perception`, `katech-autoware-system-control-arm64:adsw-control` 태그가 들어 있다. PC RViz 압축 파일에는 `katech-autoware-system-control:adsw-perception` 태그가 들어 있다.

```sh
docker save decision-ap:local-aarch64 \
  katech-autoware-system-control-arm64:adsw-perception \
  katech-autoware-system-control-arm64:adsw-control \
  | zstd -T0 -3 -q -o deployment/images/board-arm64.tar.zst
docker save katech-autoware-system-control:adsw-perception \
  | zstd -T0 -3 -q -o deployment/images/pc-rviz-amd64.tar.zst
sha256sum deployment/images/*.tar.zst deployment/map/sample-map-decision/* \
  > deployment/SHA256SUMS
```

지도를 바꾸려면 `deployment/map/sample-map-decision/`의 네 파일을 함께 교체하고 체크섬을 다시 생성한다. SDK 런타임이 포함된 이미지의 배포 범위는 해당 SDK 라이선스 조건을 따른다.
