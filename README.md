# ✈️ RADAR 360 — ESP32-S3 Realtime ADS-B Radar & Live Weather

**ESP32-4848S040 (4.0" 480×480 RGB 터치 디스플레이)** 기반의 실시간 항공기 레이더 및 기상 레이더 펌웨어입니다.  
**터치스크린 UI**와 **실시간 미리 보기 웹 콘솔**을 통해 공항, 지도, 표시 옵션을 즉시 변경할 수 있으며, **GitHub 기반 원격 OTA** 및 **로컬 `.bin` 무선 업데이트**를 지원합니다.

<!-- 실행 화면 이미지를 images 폴더에 업로드한 뒤 아래 주석을 해제하세요 -->
<!-- ![실행화면](images/실행화면.png) -->

---

## ✨ 주요 기능

### 1. 🛫 실시간 항공기 레이더 (Flight Radar)
* **실시간 ADS-B 수신:** `adsb.lol`, `adsb.fi`, `airplanes.live` API를 순차적으로 호출해 안정적으로 주변 항공기 데이터를 수신합니다.
* **초경량 배경 지도 & 디밍 처리:** 통신 부하와 화면 흔들림을 줄이기 위해 **ArcGIS Dark Gray(초경량 다크 지도)**와 **Google Roadmap(45% 어둡게 디밍 처리)**을 지원하며, 비행기 아이콘과 레이더 정보가 선명하게 돋보입니다.
* **회전형 제트기 아이콘 & 상세 오버레이:** 진행 방위각(Heading)에 따라 회전하는 비행기 아이콘, 진행 벡터, 콜사인, 기종, 고도(`ft`/`m`), 속도(`kt`/`km/h`), 거리를 표시합니다.
* **전 세계 주요 공항 프리셋:** 인천(RKSI), 부산 김해(RKPK), 제주(RKPC) 및 도쿄·싱가포르·홍콩·방콕·두바이·시드니·런던·파리·프랑크푸르트·뉴욕·LA + 위도/경도 직접 입력을 지원합니다.

### 2. 🌦️ 실시간 날씨 & 강수 레이더 (Weather View)
* **API 키 불필요:** Open-Meteo API로 현재 기온, 체감온도, 습도, 구름량, 강수량, 풍향·풍속·돌풍, 기압 및 **3일 예보(Today / Tmrw / Day 3)**를 10분마다 자동 갱신합니다.
* **RainViewer 실시간 강수 레이더:** 어둡게 처리된 배경 지도 위에 최신 비구름 레이더를 원본 밝기로 합성해 표시합니다.
* **미니 날씨 카드:** 항공기 레이더 화면 좌측 하단에도 현재 기온·습도·풍속·기압 요약 카드를 띄울 수 있습니다.

### 3. 👆 정전식 터치 UI (GT911)
* **상단 `[VIEW]` 버튼:** 항공기 레이더(Flight Radar) ↔ 실시간 날씨(Weather View) 즉시 전환.
* **상단 `[SET]` 버튼:** 3탭 터치 설정 화면(`RADAR` / `WEATHER` / `SYSTEM`)에서 지도 스타일, 반경, 단위, 표시 항목, 화면 밝기 조절 및 저장.
* **하단 지역명 터치:** 3×3 공항/지역 선택 팝업이 열리며, 선택 시 해당 지역의 지도·항공기·날씨·시간대가 즉시 반영됩니다.

### 4. 🌐 웹 콘솔 & 실시간 480×480 미리 보기
* **자동 AP 설정 모드:** Wi-Fi 미설정 또는 15초 내 연결 실패 시 `RADAR-360-Setup` 핫스팟(`http://192.168.4.1`)이 자동으로 열립니다.
* **접속 주소 및 IP 표시:** Wi-Fi 연결 후 레이더 화면 우측 하단과 웹 콘솔 `Network` 탭에서 **`http://radar.local`** 및 **기기 IP 주소**를 함께 확인할 수 있습니다.
* **LCD Live Preview:** 웹 콘솔 상단의 480×480 캔버스에서 설정 변경 결과를 저장 전에 미리 확인할 수 있습니다.

### 5. 🔄 무선 펌웨어 업데이트 (GitHub OTA / 로컬 `.bin`)
* **GitHub 원격 OTA:** 웹 콘솔 `System` 탭에서 **Check GitHub → Update Now**를 눌러 GitHub Releases에 배포된 최신 펌웨어로 무선 업데이트할 수 있습니다.
* **로컬 `.bin` 업로드:** 브라우저에서 컴파일된 `.bin` 파일을 직접 선택해 업데이트할 수 있습니다.

---

## 🛠️ 하드웨어 사양

| 항목 | 사양 |
| :--- | :--- |
| **보드** | ESP32-4848S040 (ESP32-S3) |
| **디스플레이** | 4.0인치 480×480 RGB TFT LCD (ST7701S, PCLK 6MHz 안정화) |
| **터치스크린** | GT911 정전식 터치 (I2C SDA=`19`, SCL=`45`, Addr=`0x5D`) |
| **메모리** | 16MB Flash / 8MB OPI PSRAM |
| **권장 전원** | **5V 2A 이상** 전용 어댑터 및 50cm 이하 짧은 USB 케이블 |

---

## 📦 라이브러리 설치

Arduino IDE **라이브러리 매니저**에서 아래 2개 라이브러리를 설치합니다.

1. **LovyanGFX** (by lovyan03)
2. **ArduinoJson** (by Benoit Blanchon) — *v6 및 v7 모두 호환*

---

## ⚙️ Arduino IDE 필수 보드 설정

> ⚠️ **OTA 업데이트**와 **480×480 화면 버퍼**를 위해 `PSRAM`과 `Partition Scheme` 설정이 필수입니다.

| 항목 | 설정값 | 비고 |
| :--- | :--- | :--- |
| **Board** | `ESP32S3 Dev Module` | |
| **Flash Size** | `16MB (128Mb)` | |
| **Flash Mode** | `DIO` | |
| **PSRAM** | `OPI PSRAM` | **필수** (스프라이트 및 지도 버퍼) |
| **Partition Scheme** | `16M Flash (3MB APP/9.9MB FATFS)` | **OTA 필수** (`No OTA` 선택 시 업데이트 불가) |
| **Upload Speed** | `115200` | 업로드 안정화 권장 |

---

## 🚀 GitHub OTA 버전 관리 방법

1. 소스코드 상단의 `GITHUB_VERSION_URL`을 본인 저장소의 `version.json` Raw 주소로 수정합니다.
2. 새 버전 배포 시 코드 상단의 `#define FW_VERSION "1.0.2"`처럼 **버전 번호를 올린 뒤**, Arduino IDE에서 **스케치 → 컴파일된 바이너리 내보내기**로 `.bin` 파일을 추출합니다.
3. GitHub 저장소의 **Releases**에 새 릴리스(예: `v1.0.2`)를 만들고 추출한 `.bin` 파일을 첨부합니다.
4. 저장소 최상단(`main` 브랜치)의 `version.json`을 아래와 같이 업데이트합니다. (`version` 값은 코드의 `FW_VERSION`과 `v` 없이 동일하게 맞춥니다.)

```json
{
  "version": "1.0.2",
  "url": "[https://github.com/본인아이디/저장소이름/releases/download/v1.0.2/ESP32_4848s040.ino.bin](https://github.com/본인아이디/저장소이름/releases/download/v1.0.2/ESP32_4848s040.ino.bin)"
}
