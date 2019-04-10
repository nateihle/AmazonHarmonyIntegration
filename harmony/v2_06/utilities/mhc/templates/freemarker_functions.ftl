<#--  =====================
      MACRO mhc_expand_list
      ===================== -->
<#macro mhc_expand_list list>
  <#list list as x>
    <#if x?starts_with("<#")>
      <#assign inlineTemplate = x?interpret>
      <@inlineTemplate />
    <#elseif x?contains("^")>
      <#assign y = x?replace("^","<")>
      <#assign inlineTemplate = y?interpret>
      <@inlineTemplate />
    <#else>
      ${x}
    </#if>
  </#list>
</#macro>

<#--  ===========================
      MACRO mhc_expand_list_named
      =========================== -->
<#macro mhc_expand_list_named name>
  <#assign list_name = name>
  <#if .vars[list_name]?has_content>
    <@mhc_expand_list list=.vars[list_name]/>
  </#if>
</#macro>
