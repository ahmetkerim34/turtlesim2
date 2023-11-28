#!/usr/bin/env

class def Turtlesim < ROSBase %ROSBase sinifindan turetilmis. (<) isareti c# daki (:) olarak düsünülebilir.%
    %private üye degiskenlerimiz.%
    properties (Access = 'private')
        Pub
        Sub
        Pose
    end
    
    methods(Static)
        %static metodlari burada bulunduruyoruz.%
        %this (bu örnekte turt) almaz.%
        %sadece farkindalik olsun diye koyuldu bu aralik (scope)%
    end
    
    methods
        function turt = Turtlesim(masterIP)
            %yapilandirici metod%
            %örnek t = Turtlesim('192.168.1.5') --> 192.168.1.5 deki rosmaster 'a baglanir.%
            %örnek t = Turtlesim('') --> lokalde rosmaster olarak calisir.%
            %turt burada this gibi kullaniliyor.Matlab de böyle bir kullanim tarzi var.%
            %asagidaki metodlar static olmadiklari için turt degiskenini aliyorlar.%
            %burada temel sinifin yapilandirici metodu cagiriliyor.%
            %temel sinifa ait bir metod ayni isimle türeyen sinifta var ise%
            %asagidaki bicimde cagirilir. c# da base, java da super gibi.%
            turt@ROSBase(masterIP);
        end
        
        function PoseCallback(turt, src, message)
            %turtlesim simulatoru /turtle1/pose topigini yayinlar%
            %bu topige ait mesajlari PoseCallback metodu üzerinden aliyoruz.%
            turt.Pose = message;
        end
        
        function distance = GetDistance(turt, goalPos, currentPose)
            %iki nokta arasi mesafeyi hipotenus formulu ile buluyoruz.%
			%c2 = a2 + b2%
            distance = sqrt(power((goalPos.X - currentPose.X), 2) + power((goalPos.Y - currentPose.Y), 2));
        end
        
        function CreateNode(turt, nodeName)
            %Temel sinifa ait dügüm olusturma metodunu çagiriyoruz.%
            CreateNode@ROSBase(turt, nodeName);
            pause(1);
            %üyelikleri ve yayinlari burada olusturuyoruz.%
            turt.Pub = robotics.ros.Publisher(turt.Node,'/turtle1/cmd_vel','geometry_msgs/Twist');
            turt.Sub = robotics.ros.Subscriber(turt.Node,'/turtle1/pose', @turt.PoseCallback);
        end
        
        function MoveInHeartPath(turt)
            %robota gönderecegimiz hareket mesaji%
            twist = rosmessage(turt.Pub);
            %robotun gönderdigi güncel konum bilgisi%
            goalPos = rosmessage(turt.Sub);
            %kalp sekli için gereken formül bakiniz:%
            %http://mathworld.wolfram.com/HeartCurve.html%
            size = 350;
            n = linspace(-pi,pi, size);
            %hedefe uzaklik hata payi%
            distance_tolerance = 0.1;
            
            %robotun ilk konum bigisini göndermesini bekle.%
            while (isempty(turt.Pose))
                pause(0.1);
            end
            
            %robotun yörüngesini plot fonksiyonu üzerinde diagnostik amaçli gösterebilmek için 
            %X ve Y dizileri olustur ve ilk de?erlerini sifir ata.%
            X = zeros(size);
            Y = zeros(size);
            %robotun tam olarak baslangiç pozisyonundan kalp sekli çizmesi için gereken x ve y 
            %kayiklik (ofset) degerlerini bul.%
            xShift = turt.Pose.X;
            yShift = turt.Pose.Y - ((13*cos(n(1)))-(5*cos(2*n(1)))-(2*cos(3*n(1)))-(cos(4*n(1))))/10;
            
            %her bir n esit dagilimli vektörel noktasi için x ve y degerlerini hesapla%    
            for i = 1:size
                k = n(i);
                %robotun (x,y) hedef noktasi. Hesaplamada 10'a bölmemin sebebi þeklin boyutunu küçültmek%
				%Turtlesim simülatörünün pencere sýnýrlarý basitlik için sabit tutulmuþ ve bizim bu sýnýrlarý geçmeden%
				%çizimi gerçekleþtirmemiz gerekiyor.%
                goalPos.X  = xShift + (16*(power(sin(k),3)))/10;
                goalPos.Y = yShift + ((13*cos(k))-(5*cos(2*k))-(2*cos(3*k))-(cos(4*k)))/10;
                %plot fonksiyonuna gönderece?imiz (x,y) bilgisi%
                X(i) = goalPos.X; 
                Y(i) = goalPos.Y;

                %diagnostik amaçli konum ve hedef bilgilerini matlab ekranina bas.%
                disp(fprintf('Konum(%f,%f)--->Hedef(%f,%f)', turt.Pose.X, turt.Pose.Y, goalPos.X, goalPos.Y));
                
                %mevcut konumun hedefe uzakligini hesapla (hipotenus) %
                %tolerans degeri içinde ise bir sonraki hedef noktayi ele al ve ona yönel.%
                k = 1.5;
                m = 3;
                
                while (turt.GetDistance(goalPos, turt.Pose) >= distance_tolerance)
                    %X ekseni boyunca uzaklik miktarinin k kati kadar hizlan.%
                    twist.Linear.X = k * turt.GetDistance(goalPos, turt.Pose);
                    %2 boyutlu bir simülatörde sadece z ekseninde dönme hareketi söz konusu.% 
                    %Yani robot kendi etrafinda dönebiliyor. Hedefe yönlenme acisinin (radyan cinsinden) m kati hizlan.%
                    twist.Angular.Z = m * (atan2(goalPos.Y - turt.Pose.Y, goalPos.X - turt.Pose.X) - turt.Pose.Theta);
                    %n degerindeki artisi hizin artmasina sebeb olur ve bir noktadan sonra robot savrulur%
                    %hedefi tolerans degerlerinde yakalayamadan geçmis olur ve tekrar manevra yapmaya baslar%
                    %n degerini arttirarak bunu gozlemleyebilirsiniz.%
                    
                    %matlab2015b de frekans komutlari bulunmamakta. Belli bir frekans da hiz bilgisi gönderemedigim için
                    %asagidaki gibi bir bekleme belirledim. Normalde asagidaki linkteki gibi yapmali idim.%
                    %https://www.mathworks.com/help/robotics/ug/fixed-rate-execution.html%
                    %robota konum bilgisi gönderdigimiz gibi o da kendi konumunu göndermekte.%    
                    %gönderim hizi alim hizini asar ise konum sektirmesi ve hedef noktayi geçme%
                    %söz konusu olur ve kalp sekli olusturamayiz.Pause bu durumu çözmek için konuldu.%
                    send(turt.Pub,twist);
                    pause(0.01);    
                end
                
                disp('********Hedefe ulasildi.********');
            end
            %diagnostik amaçli robotun izledigi yolun grafigini gösterelim.%
%             plot(X,Y);
            
        end
        
        function delete(turt)
            %sinifa ait yokedici metod. 
            %clear all veya clear <Turtlesim object> çagrildiginda otomatik çalisir.%
            %sinifa ait olan fonksiyonlara metod denildigini hatirlayalim.%
            disp('Turtlesim yok edici metod cagiriliyor.');   
            clear Pub;
            clear Sub;
            clear Pos;
        end
    end
end

