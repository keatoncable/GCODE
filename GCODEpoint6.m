clear
clc

%% Initialization

header = ["M73 P0 R36";
"M73 Q0 S36";
"M201 X1000 Y1000 Z1000 E5000 	; sets maximum accelerations, mm/sec^2";
"M203 X200 Y200 Z12 E120 		; sets maximum feedrates, mm/sec";
"M204 P1250 R1250 T1250 			; sets acceleration (P, T) and retract acceleration (R), mm/sec^2";
"M205 X8.00 Y8.00 Z0.40 E1.50 	; sets the jerk limits, mm/sec";
"M205 S0 T0 						; sets the minimum extruding and travel feed rate, mm/sec";
"M107							; sets fan off";
"M862.3 P ""MK3S"" 				; printer model check";
"M862.1 P0.4 					; nozzle diameter check";
"M115 U3.8.1 					; tell printer latest fw version";
"G90 							; use absolute coordinates";
"M83 							; extruder relative mode";
"M104 S215 						; set extruder temp";
"M140 S60 						; set bed temp";
"M190 S60 						; wait for bed temp";
"M109 S215 						; wait for extruder temp";
"G28 W 							; home all without mesh bed level";
"G80 							; mesh bed leveling";
"G1 Y-3.0 F1000.0 				; go outside print area";
"G92 E0.0";
"G1 X60.0 E9.0 F1000.0 			; intro line";
"M73 Q0 S36";
"M73 P0 R36";
"G1 X100.0 E12.5 F1000.0 		; intro line";
"G92 E0.0";
"M221 S95";
"G21 							; set units to millimeters";
"G90 							; use absolute coordinates";
"M83 							; use relative distances for extrusion";
"M900 K30 						; Filament gcod";
";BEFORE_LAYER_CHANGE";
"G92 E0.0";
"M106 S100						;toggle fan"];


%% Test Initial Shape
close all

gsto = ["G1 E-0.80000 F2100.00000";
        "G1 Z0.600 F10800.000";
        "G1 X100 Y100";
        "G92 X0 Y0";
        ";AFTER_LAYER_CHANGE";
        ";0.2";];

zstore = [];
mult = 0;
for i = 0.2:0.2:10
    mult = 1 + mult;
    z = [];
    for j = 1:length(0.2:0.2:10)
        zcomp = 0.2*mult;
        z = [z zcomp];
    end
zstore = [zstore ; zcomp];
end

figure 


for q = 1:length(zstore)
    hold on
    noz = 0.4;
    zcoord = zstore(q);
    x = [0.2 0.2 5.2 5.2 14.8 14.8 9.8 9.8 5.2 5.2 0.2];
    y = [5.2 19.8 14.8 19.8 19.8 5.2 5.2 0.2 0.2 5.2 5.2];
    
    height = sprintf("G1 Z%.3f",zcoord);
    gsto = [gsto; height ;"G1 X0.2 Y5.2"];
    
    sto1 = [];
   
    for w = 2:(length(x))
        x2 = [x(w-1) x(w)];
        y2 = [y(w-1) y(w)];
        if diff(x2) == 0
            e = abs(diff(y2))*0.9;
        elseif diff(y2) == 0
            e = abs(diff(x2))*0.9;
        else
            e = sum(sqrt(diff(x2).^2+diff(y2).^2))*0.9;
        end
        
        if w == 2
            save1 = string(sprintf('G1 X%.3f Y%.3f',x(1),y(1)));
            save2 = string(sprintf('G1 X%.3f Y%.3f E%.5f',x(w),y(w),e));
            sto1 = [sto1 ; save1; save2];
            
        else
            save = string(sprintf('G1 X%.3f Y%.3f E%.5f',x(w),y(w),e));
            sto1 = [sto1 ; save];
        end
    end
    
    gsto = [gsto ; sto1];
    
    xdiff = max(x)-min(x);
    ydiff = max(y)-min(y);
    
    x2 = [];
    y2 = [];
    x3 = [];
    y3 = [];
    sto1 = [];
    
    
    for i = 1:length(x)
        xref = x(i);
        yref = y(i);
        enter = 1;
        if xref == 0.2 && yref == 19.8 || xref == 5.2 && yref == 14.8
            x2 = [x2 xref+noz];
            y2 = [y2 yref-1];
            enter = 0;
        elseif xref <= xdiff/2
            x2 = [x2 xref+noz];
        else
            x2 = [x2 xref-noz];
        end
        if enter == 1
            if yref <= ydiff/2
                y2 = [y2 yref+noz];
            else
                y2 = [y2 yref-noz];
            end
        end
    end
        
    for v = 2:length(x)
        yy = [y2(v-1) y2(v)];
        xx = [x2(v-1) x2(v)];
        
        if diff(x2) == 0
            e = abs(diff(yy));
        elseif diff(y2) == 0
            e = abs(diff(xx))*0.9;
        else
            e = sum(sqrt(diff(xx).^2+diff(yy).^2))*0.9;
        end
        
        if v == 2
            save1 = string(sprintf('G1 X%.3f Y%.3f',0.6,5.6));
            save2 = string(sprintf('G1 X%.3f Y%.3f',x2(v),y2(v)));
            sto1 = [sto1 ; save1; save2];
            
        elseif v == length(x)
            save = string(sprintf('G1 X%.3f Y%.3f E%.5f',x2(v),y2(v),e));
            lift = sprintf("G1 Z%.3f",zstore(q)+1);
            sto1 = [sto1 ; save; lift];
        else
            save = string(sprintf('G1 X%.3f Y%.3f E%.5f',x2(v),y2(v),e));
            sto1 = [sto1 ; save];
        end
    end
    
    
    if q == 1 || q == 2 || q == (length(zstore)-1) || q == length(zstore)
        gsto = [gsto ; sto1];
    else
        raise = sprintf("G1 Z%.3f",zstore(q));
    end
    
    
    plot(x,y)
    plot(x2,y2)
    xlim([-1 21])
    ylim([-1 21])
    
    % FIRST BOTTOM LAYER
    if q == 1 || q == 3 || q ==length(zstore)-2 || q == length(zstore)
    
    x1y1 = [0.6 19-0.4];
    x2y2 = [5.6 14-0.4];
    v = x2y2 - x1y1;
    n = sqrt((x1y1(1)-x2y2(1))^2+((x1y1(2)-x2y2(2))^2));
    u = v/n;
    pstore = [];
    
    infx1 = [];
    infy1 = [];
    
    numlines0 = ceil((x2y2(1)-x1y1(1))/0.35);
    infline1 = ceil(numlines0);
    dist = (x2y2(1)-x1y1(1))/numlines0/sind(45);
    layer = 0.2;
    
    for i = 1:numlines0
        p1 = [x1y1(1) x1y1(2)] + dist*i*[u(1) u(2)];
        pstore = [pstore ; p1];
        if i == 1
            %plot([x1y1(1) p1(1)],[x1y1(2) p1(2)])
        else
            %plot([pstore(i-1,1) p1(1)],[pstore(i-1,2) p1(2)])
        end
    end
    
    line = x2y2(1)+0.2;
    
    sto1 = [];
    
    for i = 1:length(pstore)
        plot([pstore(i,1) pstore(i,1)],[line pstore(i,2)])
        e = (pstore(i,2)-line)*0.9;
        
        if i == 1
           lower = sprintf("G1 Z%.3f",zstore(q));
           lines = sprintf("G1 X%.3f Y%.3f",pstore(i,1),line);
           sto1 = [sto1 ; lines; lower];
        elseif i == length(pstore)
           e = pstore(i-1,2)-line;
           lines = sprintf("G1 X%.3f Y%.3f E%.5f",pstore(i-1,1),line,e);
           next = sprintf("G1 X%.3f Y%.3f",pstore(i,1),line);
           e = pstore(i,2)-line;
           last = sprintf("G1 X%.3f Y%.3f E%.5f",pstore(i,1),pstore(i,2),e);
           sto1 = [sto1 ; lines ; next ; last]; 
        elseif mod(i,2) == 0
            lines = sprintf("G1 X%.3f Y%.3f E%.5f",pstore(i-1,1),pstore(i,2),e);
            next = sprintf("G1 X%.3f Y%.3f",pstore(i,1),pstore(i,2));
            sto1 = [sto1 ; lines ; next];
        else
            lines = sprintf("G1 X%.3f Y%.3f E%.5f",pstore(i-1,1),line,e);
            next = sprintf("G1 X%.3f Y%.3f",pstore(i,1),line);
            sto1 = [sto1 ; lines ; next];
        end
    end
     
        
  gsto = [gsto ; sto1];
    
    x1 = 5.8;
    x2 = 9.2;
    y1 = 0.8;
    y2 = 19.2;
    numlines = (x2-x1)/0.35;
    dist2 = (x2-x1)/ceil(numlines);
    xsto1 = x1:dist2:x2;
    
    sto1 = [];
    
    for i = 1:length(xsto1)
        plot([xsto1(i) xsto1(i)],[y1 y2])
        e = (y2-y1)*0.9;
        if i == 1
           lines = sprintf("G1 X%.3f Y%.3f",xsto1(i),y1);
           sto1 = [sto1 ; lines];
        elseif i == 1:round(numlines)
            lines = sprintf("G1 X%.3f Y%.3f",xsto1(i),y1) ;
           sto1 = [sto1 ; lines];
        elseif mod(i,2) == 0
            lines = sprintf("G1 X%.3f Y%.3f E%.5f",xsto1(i-1),y2,e);
            next = sprintf("G1 X%.3f Y%.3f",xsto1(i),y2)
            sto1 = [sto1 ; lines ; next];
        else
            lines = sprintf("G1 X%.3f Y%.3f E%.5f",xsto1(i-1),y1,e);
            next = sprintf("G1 X%.3f Y%.3f",xsto1(i),y1);
            sto1 = [sto1 ; lines ; next];
        end
    end
    
     gsto = [gsto ; sto1];
    
    x12 = 9.4;
    x22 = 14.2;
    y12 = 5.8;
    y22 = 19.2;
    numlines2 = (x22-x12)/0.35;
    dist3 = (x22-x12)/ceil(numlines2);
    xsto2 = x12:dist3:x22;
    
    sto1 = [];
    
    for i = 1:length(xsto2)
        plot([xsto2(i) xsto2(i)],[y12 y22])
        e = abs(y12-y22)*0.9;
        if i == 1
           lines = sprintf("G1 X%.3f Y%.3f",xsto2(i),y12);
           sto1 = [sto1 ; lines];
        elseif i == 1:length(xsto2)
            lines = sprintf("G1 X%.3f Y%.3f",xsto2(i),y12); 
           sto1 = [sto1 ; lines];
        elseif mod(i,2) == 0
            lines = sprintf("G1 X%.3f Y%.3f E%.5f",xsto2(i-1),y22,e);
            next = sprintf("G1 X%.3f Y%.3f",xsto2(i),y22)
            sto1 = [sto1 ; lines ; next];
        else
            lines = sprintf("G1 X%.3f Y%.3f E%.5f",xsto2(i-1),y12,e);
            next = sprintf("G1 X%.3f Y%.3f",xsto2(i),y12)
            sto1 = [sto1 ; lines ; next];
        end
    end
    
    gsto = [gsto ; sto1];
  
    
    %% SECOND BOTTOM LAYER
    
elseif q == 2 || q == 4 || q == length(zstore)-1 || q == length(zstore)-3
    x12 = 5.8;
    x22 = 9.2;
    y12 = 0.6;
    y22 = 5.8;
    numlines2 = (y22-y12)/0.35;
    dist3 = (y22-y12)/ceil(numlines2);
    ysto2 = y12:dist3:y22;

    sto1 = [];
    for i = 1:length(ysto2)
        plot([x12 x22],[ysto2(i) ysto2(i)])
        e = abs(x12-x22)*0.4;
        if i == 1
           lower = sprintf("G1 Z%.3f",zstore(q));
           lines = sprintf("G1 X%.3f Y%.3f",x12,ysto2(i)); 
           sto1 = [sto1 ; lines; lower];
        elseif i == 1:length(ysto2)
            lines = sprintf("G1 X%.3f Y%.3f",x12,ysto2(i));
           sto1 = [sto1 ; lines];
        elseif mod(i,2) == 0
            lines = sprintf("G1 X%.3f Y%.3f E%.5f",x22,ysto2(i-1),e);
            next = sprintf("G1 X%.3f Y%.3f",x22,ysto2(i));
            sto1 = [sto1 ; lines ; next];
        else
            lines = sprintf("G1 X%.3f Y%.3f E%.5f",x12,ysto2(i-1),e);
            next = sprintf("G1 X%.3f Y%.3f",x12,ysto2(i));
            sto1 = [sto1 ; lines ; next];
        end
    end
    
    gsto = [gsto ; sto1];
    
    x12 = 0.8;
    x22 = 14.2;
    y12 = 5.8;
    y22 = 13.2;
    numlines2 = (y22-y12)/0.35;
    dist3 = (y22-y12)/ceil(numlines2);
    ysto2 = y12:dist3:y22;
    
    sto1 = [];
    for i = 1:length(ysto2)
        plot([x12 x22],[ysto2(i) ysto2(i)])
        e = abs(x12-x22)*0.4;
        if i == 1
           lines = sprintf("G1 X%.3f Y%.3f",x12,ysto2(i));
           sto1 = [sto1 ; lines];
        elseif i == 1:length(ysto2)
            lift  = sprintf("G1 Z%.3f",zstore(q)+1);
            lines = sprintf("G1 X%.3f Y%.3f",x12,ysto2(i));
           sto1 = [sto1 ; lift; lines];
           break
        elseif mod(i,2) == 0
            lines = sprintf("G1 X%.3f Y%.3f E%.5f",x22,ysto2(i-1),e);
            next = sprintf("G1 X%.3f Y%.3f",x22,ysto2(i));
            sto1 = [sto1 ; lines ; next];
        else
            lines = sprintf("G1 X%.3f Y%.3f E%.5f",x12,ysto2(i-1),e);
            next = sprintf("G1 X%.3f Y%.3f",x12,ysto2(i));
            sto1 = [sto1 ; lines ; next];
        end
    end
    
    gsto = [gsto ; sto1];
    
    x12 = 5.8;
    x22 = 14.2;
    y12 = 13.45;
    y22 = 19.4;
    numlines2 = (y22-y12)/0.35;
    dist3 = (y22-y12)/ceil(numlines2);
    ysto2 = y12:dist3:y22;
    
    for i = 1:round(numlines2)
        plot([x12 x22],[ysto2(i) ysto2(i)])
    end
    
    sto1 = [];
    for i = 1:length(ysto2)
        plot([x12 x22],[ysto2(i) ysto2(i)])
        e = abs(x12-x22)*0.4;
        if i == 1
           lower = sprintf("G1 Z%.3f",zstore(q));
           lines = sprintf("G1 X%.3f Y%.3f",x12,ysto2(i)); 
           sto1 = [sto1 ; lines; lower];
        elseif i == 1:length(ysto2)
            lines = sprintf("G1 X%.3f Y%.3f",x12,ysto2(i));
            lift  = sprintf("G1 Z%.3f",zstore(q)+1);
           sto1 = [sto1 ;lift; lines];
           break
        elseif mod(i,2) == 0
            lines = sprintf("G1 X%.3f Y%.3f E%.5f",x22,ysto2(i-1),e);
            next = sprintf("G1 X%.3f Y%.3f",x22,ysto2(i));
            sto1 = [sto1 ; lines ; next];
        else
            lines = sprintf("G1 X%.3f Y%.3f E%.5f",x12,ysto2(i-1),e);
            next = sprintf("G1 X%.3f Y%.3f",x12,ysto2(i));
            sto1 = [sto1 ; lines ; next];
        end
    end
    
    gsto = [gsto ; sto1];

    
    x1y1 = [0.6 19-0.4];
    x2y2 = [5.8 14-0.8];
    v = x2y2 - x1y1;
    n = sqrt((x1y1(1)-x2y2(1))^2+((x1y1(2)-x2y2(2))^2));
    u = v/n;
    pstore = [];
    sto1 = [];
    
    numlines0 = abs(ceil((x2y2(2)-x1y1(2))/0.35));
    dist = (x2y2(1)-x1y1(1))/numlines0/sind(45);
    layer = 0.2;
    
    xn1 = [1 13.6];
    
    for i = 1:numlines0
        p1 = [x1y1(1) x1y1(2)+0.2] + dist*i*[u(1) u(2)];
        pstore = [pstore ; p1];
        if i == 1
            %plot([x1y1(1) p1(1)],[x1y1(2) p1(2)])
        else
            %plot([pstore(i-1,1) p1(1)],[pstore(i-1,2) p1(2)])
        end
    end
    
    line = x1y1(1)+0.2;
    
    for i = 1:numlines0
        plot([line pstore(i,1)-0.2],[pstore(i,2) pstore(i,2)])
    end
   
    pstoreud = flipud(pstore);
    
     for i = 1:length(pstoreud)
        %plot([pstore(i,1) pstore(i,1)],[line pstore(i,2)])
        e = abs(pstoreud(i,1)-0.2-line)*0.4;
        
        if i == 1
           lower = sprintf("G1 Z%.3f",zstore(q));
           lines = sprintf("G1 X%.3f Y%.3f",line,pstoreud(i,2)) ;
           sto1 = [sto1 ; lines ; lower];
%         elseif i == length(pstoreud)
%            next = sprintf("G1 X%.3f Y%.3f",pstoreud(i-1,1)-0.2,pstoreud(i-1,2))
%            e = pstoreud(i-1,1)-0.2-line;
%            lines = sprintf("G1 X%.3f Y%.3f E%.5f",line,pstoreud(i-1,2),e)
% 
%            %e = pstoreud(i,2)-line;
%            %last = sprintf("G1 X%.3f Y%.3f E%.5f",pstoreud(i,2),pstoreud(i,1),e)
%            sto1 = [sto1 ; next ; lines ];%; last]; 
        elseif mod(i,2) == 0
            next = sprintf("G1 X%.3f Y%.3f",line,pstoreud(i,2));
            lines = sprintf("G1 X%.3f Y%.3f E%.5f",pstoreud(i,1)-0.2,pstoreud(i,2),e);
            sto1 = [sto1 ; next ; lines];
        else
            next = sprintf("G1 X%.3f Y%.3f",pstoreud(i,1)-0.2,pstoreud(i,2));
            lines = sprintf("G1 X%.3f Y%.3f E%.5f",line,pstoreud(i,2),e);
            sto1 = [sto1 ; next ; lines];
        end
    end
    
    gsto = [gsto ; sto1];
    
    x2 = [];
    y2 = [];
    x3 = [];
    y3 = [];
    sto1 = [];
    
    for i = 1:length(x)
        xref = x(i);
        yref = y(i);
        enter = 1;
        if xref == 0.2 && yref == 19.8 || xref == 5.2 && yref == 14.8
            x2 = [x2 xref+noz];
            y2 = [y2 yref-1];
            enter = 0;
        elseif xref <= xdiff/2
            x2 = [x2 xref+noz];
        else
            x2 = [x2 xref-noz];
        end
        if enter == 1
            if yref <= ydiff/2
                y2 = [y2 yref+noz];
            else
                y2 = [y2 yref-noz];
            end
        end
    end
    
    else
    
%     figure
%     hold on
%     plot(x,y)
%     plot(x2,y2)
%     %plot(x3,y3)
%     %plot(xref,yref)
%     xlim([-1 21])
%     ylim([-1 21])
       sto1 = [];

    for j = 5:2:11
        x2 = [];
        y2 = [];
        x3 = [];
        y3 = [];
        
        if j==7
%             x2 = [3.2 3.2 6.6 6.6 3.2];
%             y2 = [8.8 13.2 9.6 8.8 8.8];
%             plot(x2,y2)
%             for w = 2:(length(x2))
%                 x22 = [x2(w-1) x2(w)];
%                 y22 = [y2(w-1) y2(w)];
%                 if diff(x22) == 0
%                     e = abs(diff(y22));
%                 elseif diff(y22) == 0
%                     e = abs(diff(x22));
%                 else
%                     e = sum(sqrt(diff(x22).^2+diff(y22).^2));
%                 end
%                 
%                 if w == 2
%                     save1 = string(sprintf('G1 X%.3f Y%.3f',x2(1),y2(1)));
%                     save2 = string(sprintf('G1 X%.3f Y%.3f E%.5f',x2(w),y2(w),e));
%                     sto1 = [sto1 ; save1; save2];
%                     
%                 else
%                     save = string(sprintf('G1 X%.3f Y%.3f E%.5f',x2(w),y2(w),e));
%                     sto1 = [sto1 ; save];
%                 end
%             end
        elseif j ==9
%             x2 = [8.6 8.6 12 12 8.6];
%             y2 = [7.2 15.8 15.8 7.2 7.2];
%             plot(x2,y2)
%             for w = 2:(length(x2))
%                 x22 = [x2(w-1) x2(w)];
%                 y22 = [y2(w-1) y2(w)];
%                 if diff(x22) == 0
%                     e = abs(diff(y22));
%                 elseif diff(y22) == 0
%                     e = abs(diff(x22));
%                 else
%                     e = sum(sqrt(diff(x22).^2+diff(y22).^2));
%                 end
%                 
%                 if w == 2
%                     save1 = string(sprintf('G1 X%.3f Y%.3f',x2(1),y2(1)));
%                     save2 = string(sprintf('G1 X%.3f Y%.3f E%.5f',x2(w),y2(w),e));
%                     sto1 = [sto1 ; save1; save2];
%                     
%                 else
%                     save = string(sprintf('G1 X%.3f Y%.3f E%.5f',x2(w),y2(w),e));
%                     sto1 = [sto1 ; save];
%                 end
%             end
            continue
        elseif j==11
            x2 = [9 9 11.2 11.2 9];
            y2 = [9.6 15 15 9.6 9.6];
            plot(x2,y2)
            for w = 2:(length(x2))
                x22 = [x2(w-1) x2(w)];
                y22 = [y2(w-1) y2(w)];
                if diff(x22) == 0
                    e = abs(diff(y22))*0.9;
                elseif diff(y22) == 0
                    e = abs(diff(x22))*0.9;
                else
                    e = sum(sqrt(diff(x22).^2+diff(y22).^2))*0.9;
                end
                
                if w == 2
                    save1 = string(sprintf('G1 X%.3f Y%.3f',x2(1),y2(1)));
                    lower = sprintf("G1 Z%.3f",zstore(q));
                    save2 = string(sprintf('G1 X%.3f Y%.3f E%.5f',x2(w),y2(w),e));
                    sto1 = [sto1 ; save1; lower; save2];
                elseif w == length(x2)
                    save = string(sprintf('G1 X%.3f Y%.3f E%.5f',x2(w),y2(w),e));
                    raise = sprintf("G1 Z%.3f",zstore(q)+1);
                    sto1 = [sto1 ; save; raise];
                else
                    save = string(sprintf('G1 X%.3f Y%.3f E%.5f',x2(w),y2(w),e));
                    sto1 = [sto1 ; save];
                end
            end
            break
        end
        sto1 = [];
        for i = 1:length(x)
            xref = x(i);
            yref = y(i);
            multi = 0.6;
            enter = 1;
            if xref == 0.2 && yref == 19.8 || xref == 5.2 && yref == 14.8
                x2 = [x2 xref+noz*j*multi];
                y2 = [y2 yref-1.4*j*multi];
                enter = 0;
            elseif xref <= xdiff/2
                x2 = [x2 xref+noz*j*multi];
            else
                x2 = [x2 xref-noz*j*multi];
            end
            if enter == 1
                if yref <= ydiff/2
                    y2 = [y2 yref+noz*j*multi];
                else
                    y2 = [y2 yref-noz*j*multi];
                end
            end
        end
        
            for v = 2:length(x2)
                yy = [y2(v-1) y2(v)];
                xx = [x2(v-1) x2(v)];
                
                if diff(xx) == 0
                    e = abs(diff(yy))*0.9;
                elseif diff(yy) == 0
                    e = abs(diff(xx))*0.9;
                else
                    e = sum(sqrt(diff(xx).^2+diff(yy).^2))*0.9;
                end
                
                if v == 2
                    lower  = sprintf("G1 Z%.3f",zstore(q));
                    save1 = string(sprintf('G1 X%.3f Y%.3f',x2(1),y2(1)));
                    save2 = string(sprintf('G1 X%.3f Y%.3f E%.5f',x2(v),y2(v),e));
                    sto1 = [sto1 ; save1 ; lower ; save2];
                elseif v == length(x)
                    save = string(sprintf('G1 X%.3f Y%.3f',x2(v),y2(v)));
                    raise = sprintf("G1 Z%.3f",zstore(q)+1);
                    sto1 = [sto1 ; save; raise];
                else
                    save = string(sprintf('G1 X%.3f Y%.3f E%.5f',x2(v),y2(v),e));
                    sto1 = [sto1 ; save];
                end
               
            end
        plot(x2,y2)
    end
    gsto = [gsto ; sto1];
    end
end


%% Footer

footer = string(["G4 					; wait";
"M221 S100			; flow percentage";
"M104 S0 			; turn off temperature";
"M140 S0 			; turn off heatbed";
"M107 				; turn off fan";
"G1 Z50.6 			; Move print head up";
"G1 X0 Y200 F3000 	; home X axis";
"M84 				; disable motors";
"M73 P100 R0			; reset text";
"M73 Q100 S0 		; zero out text";]);

%% Compile

gcode = gsto;
gcode2 = erase(gcode,"""");
fileID = fopen('test.txt');
nbyt = fprintf(fileID,'%s',gcode2)
fclose(fileID)
%writematrix(gcode2,'test.txt')