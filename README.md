# ✈️ RADAR 360 — ESP32-S3 Realtime ADS-B Radar, Weather & Luxury Clock

**ESP32-4848S040 (4.0" 480×480 RGB 디스플레이)** 보드를 기반으로 제작된 실시간 항공기 레이더, 기상 대시보드, 그리고 명품 스타일 시계 올인원 프로젝트입니다.  
별도의 코드 수정 없이 **스마트폰/PC 웹 콘솔**에서 Wi-Fi, 전 세계 주요 공항, 구글 지도 스타일, 날씨, 시계 화면을 실시간으로 설정할 수 있습니다.

<!-- 실행 화면 이미지를 images 폴더에 업로드한 뒤 아래 경로를 맞춰주세요 -->
<!-- ![실행화면](images/preview.png) -->

---

## ✨ 주요 기능

### 1. 🛫 실시간 항공기 레이더 (Flight Radar)
* **실시간 ADS-B 데이터 수신:** `adsb.lol`, `adsb.fi`, `airplanes.live` API를 순차적으로 호출해 안정적으로 주변 항공기 정보를 가져옵니다.
* **Google Maps 배경 지도:** 선택한 공항 및 반경에 맞춰 Google Maps 타일(위성 하이브리드 / 일반 로드맵 / 지형도)을 자동 다운로드하며, 메르카토르 투영법으로 지도와 항공기 위치를 정밀하게 일치시킵니다.
* **회전형 제트기 아이콘 & 상세 정보:** 항공기의 진행 방위각(Heading)에 맞춰 회전하는 비행기 아이콘과 진행 벡터, 콜사인, 기종, 고도(ft/m), 속도(kt/km/h), 거리를 지도 위에 표시합니다.
* **전 세계 주요 공항 프리셋:**
  * **대한민국:** 인천(RKSI), 부산 김해(RKPK), 제주(RKPC)
  * **해외 주요 공항:** 도쿄 나리타/하네다, 싱가포르, 홍콩, 방콕, 두바이, 시드니, 런던, 파리, 프랑크푸르트, 뉴욕 JFK, 로스앤젤레스 + 위도/경도 직접 입력

### 2. 🌦️ 실시간 날씨 & 강수 레이더 (Weather View)
* **API 키 불필요:** Open-Meteo API를 통해 현재 기온, 체감온도, 습도, 구름량, 강수량, 풍향·풍속·돌풍, 기압 및 **3일 예보(Today / Tmrw / Day 3)**를 10분마다 자동 갱신합니다.
* **실시간 비구름 레이더:** Google Maps 배경 위에 RainViewer 실시간 강수 레이더를 합성해 표시합니다.
* **미니 날씨 카드:** 레이더 화면 좌측 하단에도 현재 기온·습도·풍속·기압 요약 카드를 함께 띄울 수 있습니다.

### 3. ⌚ 명품 아날로그 & 디지털 시계 (Clock View)
* **지역별 시간대(Timezone) 지원:** 서울, 도쿄, 싱가포르, 방콕, 두바이, 런던, 파리, 뉴욕, LA, 시드니 등 세계 주요 도시 시간을 웹에서 즉시 전환할 수 있습니다.
* **7가지 워치페이스 제공:**
  | 구분 | 스타일 | 특징 |
  | :--- | :--- | :--- |
  | **아날로그** | **ROLEX** | Submariner 블랙 다이얼 + 날짜창 |
  | **아날로그** | **LONGINES** | Master Classic 화이트 다이얼 + 블루 핸즈 |
  | **아날로그** | **BREITLING** | Navitimer 네이비 다이얼 + 3 서브 다이얼 |
  | **아날로그** | **OMEGA** | Seamaster 딥 블루 웨이브 패턴 |
  | **디지털** | **Modern** | 대형 디지털 시계 + 날짜 + 현재 기온/습도 |
  | **디지털** | **Neon Ring** | 60초 원형 네온 게이지 + 디지털 시계 |
  | **디지털** | **Retro Flip** | 클래식 플립 시계 카드 스타일 |

### 4. 🌐 내장 웹 콘솔 (Captive Portal & mDNS)
* **자동 AP 모드:** 저장된 Wi-Fi가 없거나 15초 내 연결에 실패하면 `RADAR-360-Setup` 핫스팟이 자동으로 열리며, `http://192.168.4.1`에서 주변 Wi-Fi를 스캔해 설정할 수 있습니다.
* **실시간 원격 설정:** Wi-Fi 연결 후 같은 네트워크에서 **`http://radar.local`** (또는 기기 IP)로 접속하면 재부팅 없이 화면 모드, 공항, 표시 옵션, 밝기 등을 즉시 변경할 수 있습니다.

---

## 🛠️ 하드웨어 사양

* **보드:** ESP32-4848S040 (ESP32-S3)
* **디스플레이:** 4.0인치 480×480 RGB TFT LCD (ST7701S 드라이버)
* **메모리:** 16MB Flash / 8MB OPI PSRAM

---

## 📦 라이브러리 설치

Arduino IDE의 **라이브러리 매니저**에서 아래 2개 라이브러리를 설치합니다. (나머지 `WiFi`, `WebServer`, `DNSServer`, `ESPmDNS`, `Preferences`, `HTTPClient`는 ESP32 보드 패키지에 기본 내장되어 있습니다.)

1. **LovyanGFX** (by lovyan03)
2. **ArduinoJson** (by Benoit Blanchon) — **v6 및 v7 모두 자동 호환**

---

## ⚙️ Arduino IDE 보드 설정

컴파일 및 업로드 전 **툴(Tools)** 메뉴에서 아래와 같이 설정하세요.

| 항목 | 설정값 | 비고 |
| :--- | :--- | :--- |
| **Board** | `ESP32S3 Dev Module` | ESP32 보드 패키지 3.x 권장 |
| **Flash Size** | `16MB (128Mb)` | |
| **Flash Mode** | `DIO` | 업로드 검증 오류 방지 |
| **PSRAM** | `OPI PSRAM` | **필수** (480×480 스프라이트 및 지도 버퍼용) |
| **Partition Scheme** | `Huge APP (3MB No OTA/1MB SPIFFS)` | 스케치 용량 확보 |
| **Upload Speed** | `115200` | macOS 환경 업로드 안정화 권장 |

---

## 🚀 사용 방법

1. `ESP32_4848s040.ino` 파일을 Arduino IDE에서 열고 보드에 업로드합니다.
2. 최초 부팅 시 LCD에 **RADAR 360 SETUP** 안내 화면이 표시됩니다.
3. 스마트폰이나 PC의 Wi-Fi 목록에서 **`RADAR-360-Setup`**에 연결합니다.
4. 브라우저가 자동으로 열리지 않으면 주소창에 **`http://192.168.4.1`**을 입력합니다.
5. **Network** 탭에서 `Scan networks`를 눌러 2.4GHz Wi-Fi와 비밀번호를 입력하고 **Save all settings**를 클릭합니다.
6. 재부팅 후 Wi-Fi에 연결되면 레이더가 실행되며, 이후 같은 공유기 환경에서 **`http://radar.local`**로 접속해 공항, 지도 스타일, 날씨, 시계 화면을 자유롭게 변경할 수 있습니다.

---

## 🔧 문제 해결 (Troubleshooting)

* **업로드 시 `Unable to verify flash chip connection` 오류 발생 시:**
  * **Tools → Upload Speed**를 `115200`으로, **Flash Mode**를 `DIO`로 변경하세요.
  * 보드 뒷면의 `BOOT` 버튼을 누른 상태에서 `RESET` 버튼을 눌렀다 뗀 뒤 업로드를 진행하세요.
* **Wi-Fi 연결이 안 될 때:**
  * ESP32-S3는 **2.4GHz Wi-Fi (WPA/WPA2)**만 지원합니다. 5GHz 전용 공유기에는 연결되지 않습니다.
* **화면이 검게 나오거나 `JSON ERR`가 뜰 때:**
  * **Tools → PSRAM**이 `OPI PSRAM`으로 설정되어 있는지 반드시 확인하세요.

---

## 📄 License

This project is open-source and available under the MIT License.
